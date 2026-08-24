"""Per-robot cooperative localizer — communication-as-sensing.

Wheel odometry is deliberately degraded with per-robot bias + noise (in sim
the encoders are too perfect for localization to be a real problem). Three
estimate tracks run simultaneously so every mission is its own ablation:

  A  pure dead reckoning      — integrate noisy (v, w) only; drifts unbounded
  B  compass-aided DR         — noisy v + magnetometer yaw; the no-RF baseline
  C  fused (the system)       — B's prediction + EKF range updates from the
                                RSSI of mesh packets

Track C's measurements are the mesh itself: every DIRECT (path length 1)
packet heard from a peer or a delivery-pad RF tag carries the RSSI the radio
measured. Inverting the path-loss model turns each into a range factor:
peers (their broadcast pose + covariance) give relative constraints — the
"independent yet dependent" part — while pads at known surveyed positions
anchor the whole team and stop common-mode drift. Links whose ray crosses a
known-occupied cell of the merged map are gated out (obstruction biases RSSI
long) — lidar informing RF.

The same physics is exploited in reverse for mapping: when a link reads
MORE loss than free space predicts for the estimated geometry, something is
standing between the robots. Those excess-loss rays are published on
<ns>/rf_ray and painted by the mapper as RF-shadow evidence — the mesh
mapping what lidar has not seen. Ground truth (<ns>/pose_gt) is published
for evaluation only; no behaviour consumes it.
"""
import csv
import json
import math
import os
import random

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from . import rf_model

LOC_PERIOD = 0.5        # own pose broadcast over mesh
MAX_CORRECTION = 0.6    # cap on a single EKF position update (m)
EXCESS_DB = 6.0         # excess loss above this (3-sigma of shadowing noise)
INNOV_GATE = 4.0        # chi gate in sigmas


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Localizer(Node):
    def __init__(self):
        super().__init__('localizer')
        self.declare_parameter('robot', 'rover_1')
        self.declare_parameter('spawn', [0.0, 0.0, 0.0])
        self.declare_parameter('anchors', [0.0])   # flat x,y list of pad tags
        self.declare_parameter('rf_factors', True)
        self.declare_parameter('eval_dir', '/tmp/meshbots_eval')
        self.me = self.get_parameter('robot').value
        self.spawn = list(self.get_parameter('spawn').value)
        aflat = list(self.get_parameter('anchors').value)
        self.anchors = {f'pad_{i}': (aflat[2 * i], aflat[2 * i + 1])
                        for i in range(len(aflat) // 2)}
        self.rf_on = bool(self.get_parameter('rf_factors').value)

        # Deterministic per-robot noise character.
        self.rng = random.Random(hash(self.me) & 0xffffffff)
        self.bias_v = self.rng.uniform(-0.03, 0.03)
        self.bias_w = self.rng.uniform(-0.02, 0.02)

        sx, sy, syaw = self.spawn
        self.gt = None                      # (x, y, yaw)
        self.compass = syaw
        self.dr = [sx, sy, syaw]            # track A
        self.est = np.array([sx, sy])       # tracks B/C position
        self.est_yaw = syaw
        self.P = np.eye(2) * 0.01
        self.b = [sx, sy]                   # track B (compass DR, no RF)
        self.last_odom_t = None
        self.peers = {}                     # src -> (pose, cov, stamp)
        self.grid = None                    # own merged map for link gating
        self.updates_applied = 0
        self.updates_gated = 0

        ns = f'/{self.me}'
        self.pub_pose = self.create_publisher(PoseStamped, f'{ns}/pose', 20)
        self.pub_gt = self.create_publisher(PoseStamped, f'{ns}/pose_gt', 20)
        self.pub_dr = self.create_publisher(PoseStamped, f'{ns}/pose_dr', 20)
        self.pub_ray = self.create_publisher(String, f'{ns}/rf_ray', 20)
        self.pub_mesh = self.create_publisher(String, f'{ns}/mesh/tx_app', 20)
        self.create_subscription(Odometry, f'{ns}/odom', self.on_odom, 20)
        self.create_subscription(String, f'{ns}/mesh/rx', self.on_mesh, 80)
        self.create_subscription(OccupancyGrid, f'{ns}/merged_map',
                                 self.on_map, 2)

        self.tfb = TransformBroadcaster(self)
        stfb = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f'{self.me}/base_link'
        t.child_frame_id = f'{self.me}/lidar'
        t.transform.translation.z = 0.33
        t.transform.rotation.w = 1.0
        stfb.sendTransform(t)

        self.create_timer(LOC_PERIOD, self.broadcast_loc)

        eval_dir = self.get_parameter('eval_dir').value
        self.csv = None
        if eval_dir:
            os.makedirs(eval_dir, exist_ok=True)
            self.csv = csv.writer(open(os.path.join(eval_dir, f'{self.me}.csv'),
                                       'w', newline=''))
            self.csv.writerow(['t', 'gt_x', 'gt_y',
                               'dr_x', 'dr_y',           # A: pure DR
                               'cdr_x', 'cdr_y',         # B: compass DR
                               'fused_x', 'fused_y',     # C: + RF factors
                               'P_trace'])
            self.create_timer(0.5, self.log_row)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------- prediction (odometry) ----------------

    def on_odom(self, msg: Odometry):
        sx, sy, syaw = self.spawn
        ox, oy = msg.pose.pose.position.x, msg.pose.pose.position.y
        c, s = math.cos(syaw), math.sin(syaw)
        self.gt = (sx + c * ox - s * oy, sy + s * ox + c * oy,
                   yaw_of(msg.pose.pose.orientation) + syaw)

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_odom_t is None:
            self.last_odom_t = t
            return
        dt = t - self.last_odom_t
        self.last_odom_t = t
        if dt <= 0.0 or dt > 0.5:
            return

        # What the encoders/gyro "measure": biased + noisy twist.
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        v_m = v * (1.0 + self.bias_v) + self.rng.gauss(0.0, 0.02 * abs(v) + 0.002)
        w_m = w * (1.0 + self.bias_w) + self.rng.gauss(0.0, 0.02 * abs(w) + 0.002)
        # Magnetometer: absolute yaw, noisy.
        self.compass = self.gt[2] + self.rng.gauss(0.0, 0.02)

        # Track A: pure DR.
        self.dr[2] += w_m * dt
        self.dr[0] += v_m * dt * math.cos(self.dr[2])
        self.dr[1] += v_m * dt * math.sin(self.dr[2])
        # Track B: compass DR.
        self.b[0] += v_m * dt * math.cos(self.compass)
        self.b[1] += v_m * dt * math.sin(self.compass)
        # Track C: EKF prediction with the same measured twist.
        self.est_yaw = self.compass
        self.est[0] += v_m * dt * math.cos(self.compass)
        self.est[1] += v_m * dt * math.sin(self.compass)
        q = (0.05 * abs(v_m) * dt + 0.0005) ** 2
        self.P += np.eye(2) * q

        self.publish_poses(msg.header.stamp)

    # ---------------- RF range updates ----------------

    def on_map(self, msg: OccupancyGrid):
        self.grid = (np.asarray(msg.data, dtype=np.int8)
                     .reshape(msg.info.height, msg.info.width),
                     msg.info.origin.position.x, msg.info.origin.position.y,
                     msg.info.resolution)

    def ray_blocked(self, ax, ay, bx, by):
        """True if the segment crosses a known-occupied cell of my map."""
        if self.grid is None:
            return False
        g, ox, oy, res = self.grid
        n = int(math.hypot(bx - ax, by - ay) / res) + 1
        xs = np.linspace(ax, bx, n)
        ys = np.linspace(ay, by, n)
        ix = ((xs - ox) / res).astype(int)
        iy = ((ys - oy) / res).astype(int)
        ok = (ix >= 0) & (ix < g.shape[1]) & (iy >= 0) & (iy < g.shape[0])
        return bool(np.any(g[iy[ok], ix[ok]] == 100))

    def on_mesh(self, msg: String):
        try:
            pkt = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        rf = pkt.get('rf')
        if rf is None or len(pkt.get('path', [])) != 1:
            return          # relayed packet: RSSI belongs to the last hop only
        src = pkt.get('src')
        data = pkt.get('data', {})
        if pkt.get('type') == 'ANCHOR' and src in self.anchors:
            px, py = self.anchors[src]
            self.range_update(px, py, 0.0, rf['rssi'])
        elif pkt.get('type') == 'LOC':
            p = data.get('p')
            if not p:
                return
            self.peers[src] = (p, float(data.get('pc', 1.0)), self.now())
            self.range_update(p[0], p[1], float(data.get('pc', 1.0)),
                              rf['rssi'])

    def range_update(self, px, py, peer_var, rssi):
        # RF-shadow evidence first: compare measurement with free-space
        # prediction for the *estimated* geometry.
        ex, ey = float(self.est[0]), float(self.est[1])
        d_geom = math.hypot(ex - px, ey - py)
        excess = rf_model.free_space_rssi(d_geom) - rssi
        if excess > EXCESS_DB and d_geom > 1.0:
            self.pub_ray.publish(String(data=json.dumps(
                {'a': [round(ex, 2), round(ey, 2)],
                 'b': [round(px, 2), round(py, 2)],
                 'x': round(excess, 1)})))

        if not self.rf_on:
            return
        d_hat, sigma = rf_model.rssi_to_range(rssi)
        if d_hat > 14.0 or d_geom < 0.5:
            return
        if self.ray_blocked(ex, ey, px, py):
            self.updates_gated += 1
            return

        # Scalar EKF update: z = |x - p| + noise.
        dx, dy = ex - px, ey - py
        H = np.array([dx / d_geom, dy / d_geom])
        R = sigma * sigma + peer_var
        S = float(H @ self.P @ H) + R
        innov = d_hat - d_geom
        if innov * innov > INNOV_GATE ** 2 * S:
            self.updates_gated += 1
            return
        K = (self.P @ H) / S
        step = K * innov
        norm = float(np.hypot(step[0], step[1]))
        if norm > MAX_CORRECTION:
            step *= MAX_CORRECTION / norm
        self.est += step
        self.P = (np.eye(2) - np.outer(K, H)) @ self.P
        self.P += np.eye(2) * 1e-6
        self.updates_applied += 1

    # ---------------- output ----------------

    def broadcast_loc(self):
        self.pub_mesh.publish(String(data=json.dumps({
            'type': 'LOC', 'ttl': 2,
            'data': {'p': [round(float(self.est[0]), 3),
                           round(float(self.est[1]), 3),
                           round(self.est_yaw, 3)],
                     'pc': round(float(np.trace(self.P)) / 2.0, 4)}})))

    def make_pose(self, stamp, x, y, yaw):
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        return ps

    def publish_poses(self, stamp):
        self.pub_pose.publish(self.make_pose(stamp, self.est[0], self.est[1],
                                             self.est_yaw))
        self.pub_dr.publish(self.make_pose(stamp, self.dr[0], self.dr[1],
                                           self.dr[2]))
        if self.gt is not None:
            self.pub_gt.publish(self.make_pose(stamp, *self.gt))

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = f'{self.me}/base_link'
        t.transform.translation.x = float(self.est[0])
        t.transform.translation.y = float(self.est[1])
        t.transform.translation.z = 0.15
        t.transform.rotation.z = math.sin(self.est_yaw / 2.0)
        t.transform.rotation.w = math.cos(self.est_yaw / 2.0)
        self.tfb.sendTransform(t)

    def log_row(self):
        if self.csv is None or self.gt is None:
            return
        self.csv.writerow([round(self.now(), 2),
                           round(self.gt[0], 3), round(self.gt[1], 3),
                           round(self.dr[0], 3), round(self.dr[1], 3),
                           round(self.b[0], 3), round(self.b[1], 3),
                           round(float(self.est[0]), 3),
                           round(float(self.est[1]), 3),
                           round(float(np.trace(self.P)), 5)])


def main():
    rclpy.init()
    node = Localizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
