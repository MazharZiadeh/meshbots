"""Per-robot swarm intelligence. Identical code on every rover; no central node.
Runs inside the robot's namespace; behaviour constants live in config/swarm.yaml.

All inter-robot knowledge arrives via the mesh (possibly multi-hop):

* BEACON: pose, phase, current target, delivered set, leader claim. Peers
  heard within `peer_timeout` form the "alive" set.
* Leader election: leader = lexicographically-smallest alive robot id.
  Kill the leader and, one timeout later, the next rover takes over and
  resumes the mission from its own replicated belief — decentralized failover.
* Formation (squadron): the leader drives to the objective; followers compute
  wedge slots BEHIND THE LEADER'S MESH-BEACONED POSE — their navigation goal
  literally depends on a teammate's broadcast estimate (cooperative,
  "independent yet dependent" localization). Mesh partition => followers hold.
* Task allocation: market-based. At each delivery pad every alive robot
  broadcasts a BID (cost = distance to pad + a load-balancing penalty per
  delivery already made). Everyone computes the same argmin locally, so the
  team agrees on the winner without a master. The winner detaches, docks,
  dwells, broadcasts DELIVERED, and the squadron moves on.
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray


class Swarm(Node):
    def __init__(self):
        super().__init__('swarm')
        self.me = self.get_namespace().strip('/')
        p = self.declare_parameters('', [
            ('targets', [10.0, 10.0]), ('base', [-11.0, -11.0]),
            ('beacon_period', 0.5), ('peer_timeout', 3.0),
            ('arrive_radius', 2.2), ('dock_radius', 0.55),
            ('dwell', 3.0), ('deliver_timeout', 60.0),
            ('bid_period', 1.0), ('bid_fresh', 2.6),
            ('auction_quiet', 3.0), ('load_penalty', 8.0),
            ('slots_flat', [-1.9, 1.5, -1.9, -1.5, -3.4, 0.0]),
            ('max_slot_offset', 2.5), ('eval_dir', '')])
        (targets_flat, self.base, beacon_period, self.peer_timeout,
         self.arrive_r, self.dock_r, self.dwell, self.deliver_timeout,
         self.bid_period, self.bid_fresh, self.auction_quiet,
         self.load_penalty, slots_flat, self.max_offset,
         self.eval_dir) = [x.value for x in p]
        self.targets = [(targets_flat[i], targets_flat[i + 1])
                        for i in range(0, len(targets_flat), 2)]
        self.slots = [(slots_flat[i], slots_flat[i + 1])
                      for i in range(0, len(slots_flat), 2)]

        self.pose = None                 # my (x, y, yaw)
        self.peers = {}                  # name -> {p, ph, tgt, done, t}
        self.done = set()                # delivered target indices (replicated)
        self.phase = 'TRAVEL'
        self.tgt = 0
        self.bids = {}                   # name -> (cost, stamp), current tgt
        self.deliver_since = None
        self.dock_since = None
        self.delivered_sent = 0
        self.last_bid_t = 0.0
        self.my_deliveries = 0           # load-balancing term for the auction
        self._auction_tgt = None
        self._auction_t0 = 0.0
        self.slot_offset = None          # (dx, dy) in leader frame + stamp
        self.mission_logged = False

        self.pub_mesh = self.create_publisher(String, 'mesh/tx_app', 30)
        self.pub_goal = self.create_publisher(PoseStamped, 'goal', 10)
        self.pub_slot = self.create_publisher(PoseStamped, 'slot_nominal', 10)
        self.pub_marks = self.create_publisher(MarkerArray, '/swarm/markers', 10)
        self.pub_status = self.create_publisher(String, 'swarm_status', 5)
        self.create_subscription(PoseStamped, 'pose', self.on_pose, 20)
        self.create_subscription(String, 'mesh/rx', self.on_mesh, 80)
        self.create_subscription(PointStamped, 'slot_offset',
                                 self.on_slot_offset, 10)

        self.create_timer(beacon_period, self.send_beacon)
        self.create_timer(0.2, self.tick)
        self.create_timer(1.0, self.publish_markers)

    # ---------------- helpers ----------------

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def alive(self):
        t = self.now()
        names = {self.me}
        names |= {n for n, d in self.peers.items()
                  if t - d['t'] < self.peer_timeout}
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

    def my_cost(self):
        tx, ty = self.targets[self.tgt]
        return (math.hypot(self.pose[0] - tx, self.pose[1] - ty)
                + self.load_penalty * self.my_deliveries)

    def winner(self):
        t = self.now()
        fresh = {n: c for n, (c, ts) in self.bids.items()
                 if t - ts < self.bid_fresh}
        if self.pose is not None:
            fresh[self.me] = self.my_cost()
        if not fresh:
            return None
        return min(fresh.items(), key=lambda kv: (kv[1], kv[0]))[0]

    def tick(self):
        if self.pose is None:
            return
        if self.leader() == self.me:
            self.tick_leader()
        else:
            self.tick_follower()
        self.pub_status.publish(String(data=json.dumps({
            'me': self.me, 'leader': self.leader(), 'phase': self.phase,
            'tgt': self.tgt, 'done': sorted(self.done),
            'alive': self.alive()})))
        # Evaluation hook: record when this robot learned the mission is done.
        if (not self.mission_logged and self.eval_dir
                and len(self.done) >= len(self.targets)):
            self.mission_logged = True
            os.makedirs(self.eval_dir, exist_ok=True)
            with open(os.path.join(self.eval_dir,
                                   f'mission_{self.me}.csv'), 'w') as f:
                f.write(f't_complete\n{self.now():.1f}\n')

    def tick_leader(self):
        x, y, _ = self.pose
        nxt = self.next_target()

        if self.phase == 'TRAVEL':
            if nxt is None:
                self.phase = 'RETURN'
                return
            self.tgt = nxt
            tx, ty = self.targets[self.tgt]
            if math.hypot(x - tx, y - ty) < self.arrive_r:
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
            if self.now() - self.deliver_since > self.deliver_timeout:
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
        if info is None or self.now() - info['t'] > self.peer_timeout:
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
        if self._auction_tgt != self.tgt:
            self._auction_tgt = self.tgt
            self._auction_t0 = self.now()
            self.delivered_sent = 0
            self.dock_since = None
        x, y, _ = self.pose
        tx, ty = self.targets[self.tgt]
        # Keep bidding so everyone converges on the same argmin.
        if self.now() - self.last_bid_t > self.bid_period:
            self.last_bid_t = self.now()
            self.mesh_send('BID', {'tgt': self.tgt,
                                   'c': round(self.my_cost(), 3)})
        # Quiet period: let bids propagate (possibly multi-hop) before anyone
        # acts, so early self-favouring winners don't cause churn.
        others = [n for n in self.alive() if n != self.me]
        have_all = all(n in self.bids and
                       self.now() - self.bids[n][1] < self.bid_fresh
                       for n in others)
        if not have_all and self.now() - self._auction_t0 < self.auction_quiet:
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
        if d > self.dock_r:
            self.dock_since = None
            self.publish_goal(tx, ty)
            return
        if self.dock_since is None:
            self.dock_since = self.now()
            self.get_logger().info(f'{self.me}: docked on pad {self.tgt}, delivering…')
        if self.now() - self.dock_since > self.dwell and self.delivered_sent < 3:
            if self.tgt not in self.done:
                self.my_deliveries += 1
            self.done.add(self.tgt)
            self.mesh_send('DELIVERED', {'tgt': self.tgt})
            self.delivered_sent += 1
            self.get_logger().info(f'{self.me}: DELIVERED target {self.tgt}')

    def on_slot_offset(self, msg: PointStamped):
        """Informative-formation planner's perturbation, in the leader frame."""
        self.slot_offset = (msg.point.x, msg.point.y, self.now())

    def publish_formation_goal(self, leader_info):
        p = leader_info.get('p')
        if not p:
            return
        lx, ly, lyaw = p
        followers = [n for n in self.alive() if n != self.leader()]
        try:
            slot = self.slots[followers.index(self.me)]
        except (ValueError, IndexError):
            return
        c, s = math.cos(lyaw), math.sin(lyaw)
        gx = lx + c * slot[0] - s * slot[1]
        gy = ly + s * slot[0] + c * slot[1]

        # Tell the formation planner where the nominal slot is (leader yaw in
        # the orientation), then apply its clamped perturbation if fresh.
        nom = PoseStamped()
        nom.header.stamp = self.get_clock().now().to_msg()
        nom.header.frame_id = 'map'
        nom.pose.position.x = gx
        nom.pose.position.y = gy
        nom.pose.orientation.z = math.sin(lyaw / 2.0)
        nom.pose.orientation.w = math.cos(lyaw / 2.0)
        self.pub_slot.publish(nom)

        if self.slot_offset is not None:
            dx, dy, t = self.slot_offset
            if self.now() - t < 3.0:
                norm = math.hypot(dx, dy)
                if norm > self.max_offset:
                    dx *= self.max_offset / norm
                    dy *= self.max_offset / norm
                gx += c * dx - s * dy
                gy += s * dx + c * dy
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
