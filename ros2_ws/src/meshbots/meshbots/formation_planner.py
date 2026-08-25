"""Formation-as-Aperture: informative formation perturbation (per robot).

The squadron's wedge geometry is a free variable the mission barely
constrains — so treat it as a sensing aperture. Once a second, each FOLLOWER
evaluates candidate perturbations of its nominal formation slot and scores
each candidate by the information its mesh links would yield from there:

  score(d) = alpha * loc_gain + beta * map_gain - dev_cost * |d|

* loc_gain — predicted EKF variance reduction from the RSSI range factors
  the links would produce: for each anticipated link (to the leader, to the
  other followers, to any delivery-pad RF tag in range) with predicted range
  sigma R and unit direction u, the scalar-EKF reduction (u'Pu)^2/(u'Pu+R),
  weighted by the link's predicted packet-delivery ratio. Link direction
  diversity is rewarded automatically: two orthogonal links shrink both axes
  of P; two parallel ones do not.
* map_gain — expected RF-shadow information: the number of currently-UNKNOWN
  merged-map cells the link segment crosses (whether the packet arrives clean
  or attenuated, that ray teaches us something about those cells), again
  PDR-weighted.
* dev_cost * |d| — mission-coherence price of deviating from the wedge.

Constraints: the perturbed slot must keep a usable link to the leader
(PDR >= pdr_floor), stay inside a trust region around the nominal slot, and
not sit on a known obstacle. Hysteresis avoids slot thrash. The planner is
fully decentralized: everything it uses arrives via the mesh (LOC beacons)
or local topics; it runs identical code on every rover and only followers
act. Publishing nothing (fixed formation) is the ablation baseline.
"""
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

from . import rf_model


class FormationPlanner(Node):
    def __init__(self):
        super().__init__('formation_planner')
        self.me = self.get_namespace().strip('/')
        p = self.declare_parameters('', [
            ('anchors', [0.0]),        # flat x,y of delivery-pad RF tags
            ('alpha', 1.0),            # weight: localization gain
            ('beta', 0.004),           # weight: map (RF-shadow) gain per cell
            ('dev_cost', 0.008),       # cost per metre of slot deviation
            ('trust_radius', 2.2),     # max perturbation from nominal slot
            ('pdr_floor', 0.5),        # min predicted PDR of the leader link
            ('max_link', 13.0),        # ignore links longer than this
            ('hysteresis', 1.12),      # new best must beat current by this
            ('period', 1.0)])
        (aflat, self.alpha, self.beta, self.dev_cost, self.trust_r,
         self.pdr_floor, self.max_link, self.hyst, period) = [x.value for x in p]
        self.anchors = [(aflat[2 * i], aflat[2 * i + 1])
                        for i in range(len(aflat) // 2)]

        self.P = np.eye(2) * 0.01
        self.grid = None               # (array, ox, oy, res)
        self.peers = {}                # name -> (x, y, yaw, pc, stamp)
        self.leader = None
        self.phase = None
        self.slot = None               # nominal slot: (x, y, leader_yaw, t)
        self.current = (0.0, 0.0)      # currently-published offset (leader frame)

        self.pub_offset = self.create_publisher(PointStamped, 'slot_offset', 10)
        self.create_subscription(PoseWithCovarianceStamped, 'pose_cov',
                                 self.on_cov, 10)
        self.create_subscription(OccupancyGrid, 'merged_map', self.on_map, 2)
        self.create_subscription(String, 'mesh/rx', self.on_mesh, 50)
        self.create_subscription(String, 'swarm_status', self.on_status, 10)
        self.create_subscription(PoseStamped, 'slot_nominal', self.on_slot, 10)
        self.create_timer(period, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------- inputs ----------------

    def on_cov(self, msg: PoseWithCovarianceStamped):
        c = msg.pose.covariance
        self.P = np.array([[c[0], c[1]], [c[6], c[7]]])

    def on_map(self, msg: OccupancyGrid):
        self.grid = (np.asarray(msg.data, dtype=np.int8)
                     .reshape(msg.info.height, msg.info.width),
                     msg.info.origin.position.x, msg.info.origin.position.y,
                     msg.info.resolution)

    def on_mesh(self, msg: String):
        try:
            pkt = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if pkt.get('type') != 'LOC':
            return
        src, data = pkt.get('src'), pkt.get('data', {})
        pos = data.get('p')
        if src and pos:
            self.peers[src] = (pos[0], pos[1], pos[2],
                               float(data.get('pc', 1.0)), self.now())

    def on_status(self, msg: String):
        try:
            st = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self.leader = st.get('leader')
        self.phase = st.get('phase')

    def on_slot(self, msg: PoseStamped):
        q = msg.pose.orientation
        lyaw = 2.0 * math.atan2(q.z, q.w)
        self.slot = (msg.pose.position.x, msg.pose.position.y, lyaw,
                     self.now())

    # ---------------- geometry helpers ----------------

    def cell_ok(self, x, y):
        """Candidate position must not sit on a known obstacle."""
        if self.grid is None:
            return True
        g, ox, oy, res = self.grid
        ix, iy = int((x - ox) / res), int((y - oy) / res)
        if not (0 <= ix < g.shape[1] and 0 <= iy < g.shape[0]):
            return False
        r = 3  # ~0.45 m clearance
        x0, x1 = max(0, ix - r), min(g.shape[1], ix + r + 1)
        y0, y1 = max(0, iy - r), min(g.shape[0], iy + r + 1)
        return not np.any(g[y0:y1, x0:x1] == 100)

    def link_cells(self, ax, ay, bx, by):
        """(blocked_by_known_obstacle, number_of_unknown_cells_crossed)."""
        if self.grid is None:
            return False, 0
        g, ox, oy, res = self.grid
        n = int(math.hypot(bx - ax, by - ay) / res) + 1
        xs = np.linspace(ax, bx, n)
        ys = np.linspace(ay, by, n)
        ix = ((xs - ox) / res).astype(int)
        iy = ((ys - oy) / res).astype(int)
        ok = (ix >= 0) & (ix < g.shape[1]) & (iy >= 0) & (iy < g.shape[0])
        vals = g[iy[ok], ix[ok]]
        return bool(np.any(vals == 100)), int(np.count_nonzero(vals == -1))

    # ---------------- the aperture optimization ----------------

    def link_partners(self):
        """Positions this candidate could range against: leader and other
        followers (their broadcast estimate + covariance), and pad tags
        (surveyed, zero covariance)."""
        t = self.now()
        partners = []
        for name, (x, y, _, pc, ts) in self.peers.items():
            if t - ts < 3.0:
                partners.append((x, y, pc, name == self.leader))
        for ax, ay in self.anchors:
            partners.append((ax, ay, 0.0, False))
        return partners

    def score(self, qx, qy, dev, partners):
        if not self.cell_ok(qx, qy):
            return None
        loc_gain, map_gain = 0.0, 0.0
        leader_pdr = None
        for px, py, pc, is_leader in partners:
            d = math.hypot(qx - px, qy - py)
            if d < 0.5:
                continue
            pdr_w = rf_model.pdr(rf_model.free_space_rssi(d))
            if is_leader:
                leader_pdr = pdr_w
            if d > self.max_link or pdr_w < 0.15:
                continue
            blocked, unknown = self.link_cells(qx, qy, px, py)
            if blocked:
                continue    # localizer would gate it; known-obstacle rays
                            # teach us nothing new either
            u = np.array([(qx - px) / d, (qy - py) / d])
            s = float(u @ self.P @ u)
            _, sigma = rf_model.rssi_to_range(rf_model.free_space_rssi(d))
            R = sigma * sigma + pc
            loc_gain += pdr_w * s * s / (s + R)
            map_gain += pdr_w * unknown
        if leader_pdr is not None and leader_pdr < self.pdr_floor:
            return None     # never trade away the command link
        return (self.alpha * loc_gain + self.beta * map_gain
                - self.dev_cost * dev)

    def tick(self):
        if (self.phase not in ('TRAVEL', 'RETURN') or self.slot is None
                or self.leader == self.me or self.now() - self.slot[3] > 2.0):
            return
        sx, sy, lyaw, _ = self.slot
        c, s = math.cos(lyaw), math.sin(lyaw)
        partners = self.link_partners()
        if not partners:
            return

        # Candidates: keep-current, nominal, and two rings around the slot.
        cands = [self.current, (0.0, 0.0)]
        for r in (1.0, min(2.0, self.trust_r)):
            for k in range(8):
                a = k * math.pi / 4.0
                cands.append((r * math.cos(a), r * math.sin(a)))

        best, best_score, cur_score = None, -1e9, None
        for dx, dy in cands:
            dev = math.hypot(dx, dy)
            if dev > self.trust_r:
                continue
            qx = sx + c * dx - s * dy
            qy = sy + s * dx + c * dy
            sc = self.score(qx, qy, dev, partners)
            if sc is None:
                continue
            if (dx, dy) == self.current:
                cur_score = sc
            if sc > best_score:
                best, best_score = (dx, dy), sc

        if best is None:
            return
        # Hysteresis: switch only for a clear win over the current offset.
        if cur_score is not None and best != self.current \
                and best_score < cur_score * self.hyst + 1e-6:
            best = self.current
        self.current = best

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'leader'
        out.point.x, out.point.y = float(best[0]), float(best[1])
        self.pub_offset.publish(out)


def main():
    rclpy.init()
    node = FormationPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
