"""Per-robot collaborative occupancy mapper (C-SLAM-lite) + RF-shadow layer.
Runs inside the robot's namespace; grid geometry and sharing behaviour live
in config/mapping.yaml.

Builds a log-odds occupancy grid by raytracing the lidar from the FUSED pose
estimate published by the localizer (not ground truth — map quality honestly
degrades with localization error).

Map sharing happens EXCLUSIVELY over the mesh: once a second the mapper
broadcasts the cells whose state changed ("map patches"). Patches from peers
— possibly relayed multi-hop — are merged into a fused map. Direct
observation wins over hearsay: a cell this robot sensed itself is never
overwritten by a peer. If the mesh partitions, the fused maps visibly stop
converging; that is the point.

ISAC addition — the mesh maps what lidar has not seen: the localizer emits
rf_ray events for links that measured more attenuation than free space
predicts. Cells along those inter-robot rays accumulate obstruction
evidence; where lidar knows nothing but RF evidence is strong, the merged
map shows a "suspected obstacle" and the layer is published raw on rf_map.
"""
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

L_HIT = 6
L_FREE = -2
L_MIN, L_MAX = -40, 60
OCC_T = 8
FREE_T = -4


class Mapper(Node):
    def __init__(self):
        super().__init__('mapper')
        self.me = self.get_namespace().strip('/')
        p = self.declare_parameters('', [
            ('resolution', 0.15), ('size_m', 32.0), ('origin', -16.0),
            ('scan_period', 0.3), ('patch_cell_cap', 3000),
            ('rf_vote_threshold', 4.0), ('suspect_value', 60)])
        (self.res, size_m, self.origin, self.scan_period, self.patch_cap,
         self.rf_vote_t, self.suspect) = [x.value for x in p]
        self.size = int(size_m / self.res)

        self.logodds = np.zeros((self.size, self.size), dtype=np.int16)
        self.state = np.full((self.size, self.size), -1, dtype=np.int8)
        self.peer_maps = {}
        self.rf_votes = np.zeros((self.size, self.size), dtype=np.float32)
        self.dirty = set()
        self.pose = None
        self.last_scan_t = 0.0

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_map = self.create_publisher(OccupancyGrid, 'map', latched)
        self.pub_merged = self.create_publisher(OccupancyGrid, 'merged_map', latched)
        self.pub_rf = self.create_publisher(OccupancyGrid, 'rf_map', latched)
        self.pub_mesh = self.create_publisher(String, 'mesh/tx_app', 20)

        self.create_subscription(PoseStamped, 'pose', self.on_pose, 20)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 5)
        self.create_subscription(String, 'mesh/rx', self.on_mesh, 50)
        self.create_subscription(String, 'rf_ray', self.on_rf_ray, 20)

        self.create_timer(1.0, self.broadcast_patch)
        self.create_timer(1.0, self.publish_maps)

    def on_pose(self, msg: PoseStamped):
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    # ---------------- lidar mapping ----------------

    def on_scan(self, msg: LaserScan):
        if self.pose is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_scan_t < self.scan_period:
            return
        self.last_scan_t = now

        size = self.size
        x, y, yaw = self.pose
        rx = int((x - self.origin) / self.res)
        ry = int((y - self.origin) / self.res)
        if not (0 <= rx < size and 0 <= ry < size):
            return

        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = len(ranges)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment + yaw
        max_r = msg.range_max

        touched = {}
        for i in range(0, n, 2):
            r = ranges[i]
            hit = True
            if not np.isfinite(r) or r >= max_r * 0.99:
                r = max_r * 0.95
                hit = False
            elif r < msg.range_min:
                continue
            ex = x + r * math.cos(angles[i])
            ey = y + r * math.sin(angles[i])
            gx = int((ex - self.origin) / self.res)
            gy = int((ey - self.origin) / self.res)
            steps = max(abs(gx - rx), abs(gy - ry))
            if steps == 0:
                continue
            xs = np.rint(np.linspace(rx, gx, steps + 1)).astype(np.int32)
            ys = np.rint(np.linspace(ry, gy, steps + 1)).astype(np.int32)
            ok = (xs >= 0) & (xs < size) & (ys >= 0) & (ys < size)
            xs, ys = xs[ok], ys[ok]
            if len(xs) == 0:
                continue
            if hit:
                fxs, fys = xs[:-1], ys[:-1]
                touched[(int(ys[-1]), int(xs[-1]))] = L_HIT
            else:
                fxs, fys = xs, ys
            for fx, fy in zip(fxs, fys):
                key = (int(fy), int(fx))
                if key not in touched:
                    touched[key] = L_FREE

        for (cy, cx), delta in touched.items():
            old = int(self.state[cy, cx])
            self.logodds[cy, cx] = np.clip(self.logodds[cy, cx] + delta, L_MIN, L_MAX)
            new = self.tri(self.logodds[cy, cx])
            if new != old:
                self.state[cy, cx] = new
                self.dirty.add(cy * size + cx)

    @staticmethod
    def tri(lo):
        if lo > OCC_T:
            return 100
        if lo < FREE_T:
            return 0
        return -1

    # ---------------- RF-shadow evidence ----------------

    def on_rf_ray(self, msg: String):
        try:
            ray = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        (ax, ay), (bx, by) = ray['a'], ray['b']
        excess = float(ray.get('x', 0.0))
        length = math.hypot(bx - ax, by - ay)
        if length < 1.5:
            return
        # Skip the segment ends (the robots themselves occupy those cells).
        trim = min(0.7 / length, 0.4)
        n = int(length / self.res) + 1
        ts = np.linspace(trim, 1.0 - trim, n)
        xs = ax + ts * (bx - ax)
        ys = ay + ts * (by - ay)
        ix = ((xs - self.origin) / self.res).astype(int)
        iy = ((ys - self.origin) / self.res).astype(int)
        ok = (ix >= 0) & (ix < self.size) & (iy >= 0) & (iy < self.size)
        self.rf_votes[iy[ok], ix[ok]] += min(excess, 12.0) / 8.0

    # ---------------- mesh map sharing ----------------

    def broadcast_patch(self):
        if not self.dirty:
            return
        batch = []
        while self.dirty and len(batch) < self.patch_cap:
            batch.append(self.dirty.pop())
        occ = [i for i in batch if self.state.flat[i] == 100]
        free = [i for i in batch if self.state.flat[i] == 0]
        payload = {'type': 'MAP_PATCH', 'ttl': 4,
                   'data': {'o': occ, 'f': free}}
        self.pub_mesh.publish(String(data=json.dumps(payload)))

    def on_mesh(self, msg: String):
        try:
            pkt = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if pkt.get('type') != 'MAP_PATCH':
            return
        src = pkt.get('src')
        if src is None or src == self.me:
            return
        grid = self.peer_maps.get(src)
        if grid is None:
            grid = np.full((self.size, self.size), -1, dtype=np.int8)
            self.peer_maps[src] = grid
        data = pkt.get('data', {})
        ncells = self.size * self.size
        for i in data.get('o', []):
            if 0 <= i < ncells:
                grid.flat[i] = 100
        for i in data.get('f', []):
            if 0 <= i < ncells:
                grid.flat[i] = 0

    # ---------------- publishing ----------------

    def grid_msg(self, arr):
        g = OccupancyGrid()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'map'
        g.info.resolution = self.res
        g.info.width = self.size
        g.info.height = self.size
        g.info.origin.position.x = self.origin
        g.info.origin.position.y = self.origin
        g.info.origin.orientation.w = 1.0
        g.data = arr.astype(np.int8).ravel().tolist()
        return g

    def publish_maps(self):
        self.pub_map.publish(self.grid_msg(self.state))
        merged = self.state.copy()
        for grid in self.peer_maps.values():
            unknown = merged == -1
            merged[unknown] = grid[unknown]
        # RF-shadow: where nothing lidar-based is known but the mesh kept
        # losing signal through it, mark a suspected obstacle.
        suspect = (merged == -1) & (self.rf_votes >= self.rf_vote_t)
        merged[suspect] = self.suspect
        self.pub_merged.publish(self.grid_msg(merged))

        rf = np.full((self.size, self.size), -1, dtype=np.int8)
        hot = self.rf_votes > 0.5
        rf[hot] = np.clip(self.rf_votes[hot] * 20.0, 1, 100).astype(np.int8)
        self.pub_rf.publish(self.grid_msg(rf))


def main():
    rclpy.init()
    node = Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
