"""Per-robot swarm intelligence. Identical code on every rover; no central node.

All inter-robot knowledge arrives via the mesh (possibly multi-hop):

* BEACON (2 Hz): pose, phase, current target, delivered set, leader claim.
  Peers heard within `peer_timeout` form the "alive" set.
* Leader election: leader = lexicographically-smallest alive robot id.
  Kill the leader and, one timeout later, the next rover takes over and
  resumes the mission from its own replicated belief — decentralized failover.
* Formation (squadron): the leader drives to the objective; followers compute
  wedge slots BEHIND THE LEADER'S MESH-BEACONED POSE — their navigation goal
  literally depends on a teammate's broadcast estimate (cooperative,
  "independent yet dependent" localization). Mesh partition => followers hold.
* Task allocation: market-based. At each delivery pad every alive robot
  broadcasts a BID (cost = its distance to the pad). Everyone computes the
  same argmin locally => everyone agrees on the winner without a master.
  The winner detaches, docks on the pad, dwells, broadcasts DELIVERED,
  and the squadron moves on.
"""
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

BEACON_PERIOD = 0.5
TICK_PERIOD = 0.2
PEER_TIMEOUT = 3.0
BID_PERIOD = 1.0
BID_FRESH = 2.6
ARRIVE_R = 2.2          # squadron "at objective" radius (leader)
DOCK_R = 0.55           # deliverer docking radius
DWELL = 3.0             # seconds spent "delivering"
DELIVER_TIMEOUT = 60.0
SLOTS = [(-1.9, 1.5), (-1.9, -1.5), (-3.4, 0.0)]  # wedge offsets in leader frame


class Swarm(Node):
    def __init__(self):
        super().__init__('swarm')
        self.declare_parameter('robot', 'rover_1')
        self.declare_parameter('targets', [10.0, 10.0])   # flat x,y list
        self.declare_parameter('base', [-11.0, -11.0])
        self.me = self.get_parameter('robot').value
        flat = list(self.get_parameter('targets').value)
        self.targets = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
        self.base = list(self.get_parameter('base').value)

        self.pose = None                 # my (x, y, yaw)
        self.peers = {}                  # name -> {p, ph, tgt, done, t}
        self.done = set()                # delivered target indices (replicated)
        self.phase = 'TRAVEL'            # my belief of squadron phase
        self.tgt = 0                     # current target index
        self.bids = {}                   # name -> (cost, stamp), for current tgt
        self.deliver_since = None
        self.dock_since = None
        self.delivered_sent = 0
        self.last_bid_t = 0.0
        self.my_deliveries = 0   # load-balancing term for the auction

        ns = f'/{self.me}'
        self.pub_mesh = self.create_publisher(String, f'{ns}/mesh/tx_app', 30)
        self.pub_goal = self.create_publisher(PoseStamped, f'{ns}/goal', 10)
        self.pub_marks = self.create_publisher(MarkerArray, '/swarm/markers', 10)
        self.pub_status = self.create_publisher(String, f'{ns}/swarm_status', 5)
        self.create_subscription(PoseStamped, f'{ns}/pose', self.on_pose, 20)
        self.create_subscription(String, f'{ns}/mesh/rx', self.on_mesh, 80)

        self.create_timer(BEACON_PERIOD, self.send_beacon)
        self.create_timer(TICK_PERIOD, self.tick)
        self.create_timer(1.0, self.publish_markers)

    # ---------------- helpers ----------------

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def alive(self):
        t = self.now()
        names = {self.me}
        names |= {n for n, d in self.peers.items() if t - d['t'] < PEER_TIMEOUT}
        return sorted(names)

    def leader(self):
        return self.alive()[0]

    def mesh_send(self, mtype, data, ttl=4):
        self.pub_mesh.publish(String(data=json.dumps(
            {'type': mtype, 'ttl': ttl, 'data': data})))

    # ---------------- mesh I/O ----------------

    def send_beacon(self):
        if self.pose is None:
            return
        self.mesh_send('BEACON', {
            'p': [round(v, 3) for v in self.pose],
            'ph': self.phase, 'tgt': self.tgt,
            'done': sorted(self.done), 'ldr': self.leader()})

    def on_mesh(self, msg: String):
        try:
            pkt = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        src, mtype, data = pkt.get('src'), pkt.get('type'), pkt.get('data', {})
        if src == self.me:
            return
        if mtype == 'BEACON':
            self.peers[src] = {'p': data.get('p'), 'ph': data.get('ph'),
                               'tgt': data.get('tgt', 0),
                               'done': set(data.get('done', [])),
                               't': self.now()}
            self.done |= self.peers[src]['done']
        elif mtype == 'BID':
            if data.get('tgt') == self.tgt:
                self.bids[src] = (data.get('c', 1e9), self.now())
        elif mtype == 'DELIVERED':
            self.done.add(int(data.get('tgt', -1)))

    # ---------------- pose ----------------

    def on_pose(self, msg: PoseStamped):
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    # ---------------- mission logic ----------------

    def next_target(self):
        for i in range(len(self.targets)):
            if i not in self.done:
                return i
        return None

    def winner(self):
        t = self.now()
        fresh = {n: c for n, (c, ts) in self.bids.items() if t - ts < BID_FRESH}
        if self.pose is not None:
            tx, ty = self.targets[self.tgt]
            fresh[self.me] = (math.hypot(self.pose[0] - tx, self.pose[1] - ty)
                              + 8.0 * self.my_deliveries)
        if not fresh:
            return None
        return min(fresh.items(), key=lambda kv: (kv[1], kv[0]))[0]

    def tick(self):
        if self.pose is None:
            return
        me_leader = self.leader() == self.me

        if me_leader:
            self.tick_leader()
        else:
            self.tick_follower()

        self.pub_status.publish(String(data=json.dumps({
            'me': self.me, 'leader': self.leader(), 'phase': self.phase,
            'tgt': self.tgt, 'done': sorted(self.done),
            'alive': self.alive()})))

    def tick_leader(self):
        x, y, _ = self.pose
        nxt = self.next_target()

        if self.phase == 'TRAVEL':
            if nxt is None:
                self.phase = 'RETURN'
                return
            self.tgt = nxt
            tx, ty = self.targets[self.tgt]
            if math.hypot(x - tx, y - ty) < ARRIVE_R:
                self.phase = 'DELIVER'
                self.deliver_since = self.now()
                self.bids = {}
            else:
                self.publish_goal(tx, ty)

        elif self.phase == 'DELIVER':
            if self.tgt in self.done:
                self.phase = 'TRAVEL'
                self.dock_since = None
                self.delivered_sent = 0
                return
            if self.now() - self.deliver_since > DELIVER_TIMEOUT:
                self.deliver_since = self.now()
                self.bids = {}
            self.run_auction_role()

        elif self.phase == 'RETURN':
            if nxt is not None:          # a straggler target reappeared
                self.phase = 'TRAVEL'
                return
            bx, by = self.base
            if math.hypot(x - bx, y - by) < 2.0:
                self.phase = 'DONE'
            else:
                self.publish_goal(bx, by)

        # DONE: publish nothing; navigator stops on stale goal.

    def tick_follower(self):
        ldr = self.leader()
        info = self.peers.get(ldr)
        if info is None or self.now() - info['t'] > PEER_TIMEOUT:
            return                        # partitioned from leader: hold
        # Adopt the leader's replicated mission state.
        self.phase = info['ph'] or self.phase
        self.tgt = info['tgt']

        if self.phase in ('TRAVEL', 'RETURN', 'DONE'):
            self.publish_formation_goal(info)
        elif self.phase == 'DELIVER':
            self.run_auction_role(leader_info=info)

    def run_auction_role(self, leader_info=None):
        """Common DELIVER-phase behaviour for leader and followers."""
        if self.tgt in self.done or self.tgt >= len(self.targets):
            return
        if getattr(self, '_auction_tgt', None) != self.tgt:
            self._auction_tgt = self.tgt
            self._auction_t0 = self.now()
            self.delivered_sent = 0
            self.dock_since = None
        x, y, _ = self.pose
        tx, ty = self.targets[self.tgt]
        # Keep bidding so everyone converges on the same argmin.
        if self.now() - self.last_bid_t > BID_PERIOD:
            self.last_bid_t = self.now()
            cost = math.hypot(x - tx, y - ty) + 8.0 * self.my_deliveries
            self.mesh_send('BID', {'tgt': self.tgt, 'c': round(cost, 3)})
        # Quiet period: let bids propagate (possibly multi-hop) before anyone
        # acts, so early self-favouring winners don't cause churn.
        others = [n for n in self.alive() if n != self.me]
        have_all = all(n in self.bids and
                       self.now() - self.bids[n][1] < BID_FRESH for n in others)
        if not have_all and self.now() - self._auction_t0 < 3.0:
            if leader_info is not None:
                self.publish_formation_goal(leader_info)
            return
        w = self.winner()
        if w != self.me:
            self.dock_since = None
            if leader_info is not None:
                self.publish_formation_goal(leader_info)
            return
        # I won the auction: dock on the pad and deliver.
        d = math.hypot(x - tx, y - ty)
        if d > DOCK_R:
            self.dock_since = None
            self.publish_goal(tx, ty)
            return
        if self.dock_since is None:
            self.dock_since = self.now()
            self.get_logger().info(f'{self.me}: docked on pad {self.tgt}, delivering…')
        if self.now() - self.dock_since > DWELL and self.delivered_sent < 3:
            if self.tgt not in self.done:
                self.my_deliveries += 1
            self.done.add(self.tgt)
            self.mesh_send('DELIVERED', {'tgt': self.tgt})
            self.delivered_sent += 1
            self.get_logger().info(f'{self.me}: DELIVERED target {self.tgt}')
        if self.tgt in self.done:
            self.delivered_sent = 0

    def publish_formation_goal(self, leader_info):
        p = leader_info.get('p')
        if not p:
            return
        lx, ly, lyaw = p
        alive = self.alive()
        followers = [n for n in alive if n != self.leader()]
        try:
            slot = SLOTS[followers.index(self.me)]
        except ValueError:
            return
        c, s = math.cos(lyaw), math.sin(lyaw)
        gx = lx + c * slot[0] - s * slot[1]
        gy = ly + s * slot[0] + c * slot[1]
        self.publish_goal(gx, gy)

    def publish_goal(self, gx, gy):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'map'
        g.pose.position.x = float(gx)
        g.pose.position.y = float(gy)
        g.pose.orientation.w = 1.0
        self.pub_goal.publish(g)

    # ---------------- visualization ----------------

    def publish_markers(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        # Only the current leader draws the shared mission markers.
        if self.leader() == self.me:
            for i, (tx, ty) in enumerate(self.targets):
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = stamp
                m.ns = 'targets'
                m.id = i
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.pose.position.x = tx
                m.pose.position.y = ty
                m.pose.position.z = 0.15
                m.scale = Vector3(x=2.0, y=2.0, z=0.3)
                if i in self.done:
                    m.color = ColorRGBA(r=0.1, g=0.85, b=0.25, a=0.75)
                else:
                    m.color = ColorRGBA(r=0.9, g=0.15, b=0.15, a=0.75)
                arr.markers.append(m)
        if self.pose is not None:
            role = 'LEADER' if self.leader() == self.me else 'follower'
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = f'label_{self.me}'
            m.id = 0
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = self.pose[0]
            m.pose.position.y = self.pose[1]
            m.pose.position.z = 1.1
            m.scale.z = 0.45
            m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            m.text = f'{self.me} [{role}] {self.phase}'
            arr.markers.append(m)
        self.pub_marks.publish(arr)


def main():
    rclpy.init()
    node = Swarm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
