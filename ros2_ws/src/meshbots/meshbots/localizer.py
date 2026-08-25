"""Per-robot cooperative localizer — communication-as-sensing.
Runs inside the robot's namespace; noise character and EKF tuning live in
config/localization.yaml.

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
standing between the robots. Those excess-loss rays are published on rf_ray
and painted by the mapper as RF-shadow evidence — the mesh mapping what
lidar has not seen. Ground truth (pose_gt) is published for evaluation
only; no behaviour consumes it.
"""
import csv
import json
import math
import os
import random
import zlib

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                               TransformStamped)
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from . import rf_model


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Localizer(Node):
    def __init__(self):
        super().__init__('localizer')
        self.me = self.get_namespace().strip('/')
        p = self.declare_parameters('', [
            ('spawn', [0.0, 0.0, 0.0]),
            ('anchors', [0.0]),          # flat x,y list of pad tags
            ('rf_factors', True),
            ('eval_dir', '/tmp/meshbots_eval'),
            ('noise_seed', 0),
            ('loc_period', 0.5),
            ('bias_v_min', 0.04), ('bias_v_max', 0.09), ('bias_w_max', 0.03),
            ('sigma_v', 0.04), ('sigma_w', 0.03), ('compass_sigma', 0.02),
            ('process_gain', 0.09), ('max_correction', 0.6),
            ('innovation_gate', 4.0), ('max_range_factor', 14.0),
            ('excess_db', 6.0),
            ('peer_inflation', 2.0),   # inflate peer-factor R: our fusion
                                       # double-counts correlated peer info
            ('p_floor', 0.02)])        # covariance floor against the
                                       # overconfidence that locks in bias
        (self.spawn, aflat, self.rf_on, eval_dir, noise_seed, loc_period,
         bias_v_min, bias_v_max, bias_w_max, self.sigma_v, self.sigma_w,
         self.compass_sigma, self.process_gain, self.max_correction,
         self.innov_gate, self.max_range, self.excess_db,
         self.peer_inflation, self.p_floor) = [x.value for x in p]
        self.anchors = {f'pad_{i}': (aflat[2 * i], aflat[2 * i + 1])
                        for i in range(len(aflat) // 2)}

        # Deterministic per-robot noise character (crc32, not hash():
        # Python string hashes are randomized per process, which would make
        # every run draw different biases and break reproducibility).
        # noise_seed varies the draw per Monte Carlo run while staying
        # reproducible: same (robot, seed) -> same noise character.
        self.rng = random.Random(
            zlib.crc32(self.me.encode()) ^ (int(noise_seed) * 0x9E3779B1))
        self.bias_v = (self.rng.uniform(bias_v_min, bias_v_max)
                       * self.rng.choice([-1, 1]))
        self.bias_w = self.rng.uniform(-bias_w_max, bias_w_max)

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

        self.pub_pose = self.create_publisher(PoseStamped, 'pose', 20)
        self.pub_cov = self.create_publisher(PoseWithCovarianceStamped,
                                             'pose_cov', 10)
        self.pub_gt = self.create_publisher(PoseStamped, 'pose_gt', 20)
        self.pub_dr = self.create_publisher(PoseStamped, 'pose_dr', 20)
        self.pub_ray = self.create_publisher(String, 'rf_ray', 20)
        self.pub_mesh = self.create_publisher(String, 'mesh/tx_app', 20)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_subscription(String, 'mesh/rx', self.on_mesh, 80)
        self.create_subscription(OccupancyGrid, 'merged_map', self.on_map, 2)

        self.tfb = TransformBroadcaster(self)
        stfb = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f'{self.me}/base_link'
        t.child_frame_id = f'{self.me}/lidar'
        t.transform.translation.z = 0.33
        t.transform.rotation.w = 1.0
        stfb.sendTransform(t)

        self.create_timer(loc_period, self.broadcast_loc)

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
        v_m = (v * (1.0 + self.bias_v)
               + self.rng.gauss(0.0, self.sigma_v * abs(v) + 0.003))
        w_m = (w * (1.0 + self.bias_w)
               + self.rng.gauss(0.0, self.sigma_w * abs(w) + 0.003))
        # Magnetometer: absolute yaw, noisy.
        self.compass = self.gt[2] + self.rng.gauss(0.0, self.compass_sigma)

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
        q = (self.process_gain * abs(v_m) * dt + 0.001) ** 2
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
        if excess > self.excess_db and d_geom > 1.0:
            self.pub_ray.publish(String(data=json.dumps(
                {'a': [round(ex, 2), round(ey, 2)],
                 'b': [round(px, 2), round(py, 2)],
                 'x': round(excess, 1)})))

        if not self.rf_on:
            return
        d_hat, sigma = rf_model.rssi_to_range(rssi)
        if d_hat > self.max_range or d_geom < 0.5:
            return
        if self.ray_blocked(ex, ey, px, py):
            self.updates_gated += 1
            return

        # Scalar EKF update: z = |x - p| + noise. Peer factors get their R
        # inflated: peer estimates are correlated with ours (they were
        # partly built from our own broadcasts), and treating them as
        # independent double-counts information.
        dx, dy = ex - px, ey - py
        H = np.array([dx / d_geom, dy / d_geom])
        R = sigma * sigma + self.peer_inflation * peer_var
        S = float(H @ self.P @ H) + R
        innov = d_hat - d_geom
        if innov * innov > self.innov_gate ** 2 * S:
            self.updates_gated += 1
            return
        K = (self.P @ H) / S
        step = K * innov
        norm = float(np.hypot(step[0], step[1]))
        if norm > self.max_correction:
            step *= self.max_correction / norm
        self.est += step
        self.P = (np.eye(2) - np.outer(K, H)) @ self.P
        # Covariance floor: with correlated updates the filter otherwise
        # grows overconfident and a locked-in bias becomes uncorrectable.
        self.P[0, 0] = max(self.P[0, 0], self.p_floor)
        self.P[1, 1] = max(self.P[1, 1], self.p_floor)
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
        pc = PoseWithCovarianceStamped()
        pc.header.stamp = stamp
        pc.header.frame_id = 'map'
        pc.pose.pose = self.make_pose(stamp, self.est[0], self.est[1],
                                      self.est_yaw).pose
        pc.pose.covariance[0] = float(self.P[0, 0])
        pc.pose.covariance[1] = float(self.P[0, 1])
        pc.pose.covariance[6] = float(self.P[1, 0])
        pc.pose.covariance[7] = float(self.P[1, 1])
        self.pub_cov.publish(pc)
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
