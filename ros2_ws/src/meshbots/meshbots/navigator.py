"""Per-robot reactive navigator.

Potential-field local planner: attractive pull toward the current goal,
repulsive push from lidar returns, converted to unicycle (v, w) commands.
Teammates are avoided for free — they show up on lidar like any obstacle.
Stops when the goal goes stale (>1.5 s without a refresh) so higher layers
"hold" a robot simply by not publishing; includes a wiggle-out stuck escape.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan

GOAL_STALE = 1.5
STOP_DIST = 0.35
V_MAX = 0.8
W_MAX = 1.8
REPULSE_R = 1.6
REPULSE_K = 1.4
STUCK_WINDOW = 5.0
STUCK_DIST = 0.08
WIGGLE_T = 1.6


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.declare_parameter('robot', 'rover_1')
        self.me = self.get_parameter('robot').value

        self.pose = None
        self.goal = None
        self.goal_t = 0.0
        self.scan = None
        self.track = []            # (t, x, y) history for stuck detection
        self.wiggle_until = 0.0
        self.wiggle_sign = 1.0

        ns = f'/{self.me}'
        self.pub_cmd = self.create_publisher(Twist, f'{ns}/cmd_vel', 10)
        self.create_subscription(PoseStamped, f'{ns}/pose', self.on_pose, 20)
        self.create_subscription(PoseStamped, f'{ns}/goal', self.on_goal, 10)
        self.create_subscription(LaserScan, f'{ns}/scan', self.on_scan, 5)
        self.create_timer(0.1, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg: PoseStamped):
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def on_goal(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_t = self.now()

    def on_scan(self, msg: LaserScan):
        self.scan = msg

    def stop(self):
        self.pub_cmd.publish(Twist())
        self.track = []

    def tick(self):
        t = self.now()
        if (self.pose is None or self.goal is None
                or t - self.goal_t > GOAL_STALE):
            self.stop()
            return

        x, y, yaw = self.pose
        gx, gy = self.goal
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist < STOP_DIST:
            self.stop()
            return

        # Stuck detection -> reverse-and-turn wiggle.
        self.track.append((t, x, y))
        self.track = [p for p in self.track if t - p[0] < STUCK_WINDOW]
        if t < self.wiggle_until:
            cmd = Twist()
            cmd.linear.x = -0.25
            cmd.angular.z = self.wiggle_sign * 1.2
            self.pub_cmd.publish(cmd)
            return
        if len(self.track) > 20 and t - self.track[0][0] > STUCK_WINDOW * 0.9:
            moved = math.hypot(x - self.track[0][1], y - self.track[0][2])
            if moved < STUCK_DIST:
                self.wiggle_until = t + WIGGLE_T
                self.wiggle_sign = -self.wiggle_sign
                self.track = []
                return

        # Attractive force toward goal, in the robot body frame.
        goal_bearing = math.atan2(dy, dx) - yaw
        mag = min(1.0, dist / 2.0)
        fx = mag * math.cos(goal_bearing)
        fy = mag * math.sin(goal_bearing)

        # Repulsive force from lidar returns (body frame).
        min_clear = float('inf')
        if self.scan is not None:
            r = np.asarray(self.scan.ranges, dtype=np.float32)
            a = self.scan.angle_min + np.arange(len(r)) * self.scan.angle_increment
            valid = np.isfinite(r) & (r > self.scan.range_min)
            near = valid & (r < REPULSE_R)
            if near.any():
                rr, aa = r[near], a[near]
                min_clear = float(rr.min())
                w = REPULSE_K * (REPULSE_R - rr) ** 2 / (rr + 0.05) / len(rr)
                fx += float(np.sum(-w * np.cos(aa)))
                fy += float(np.sum(-w * np.sin(aa)))

        heading_err = math.atan2(fy, fx)
        cmd = Twist()
        cmd.angular.z = max(-W_MAX, min(W_MAX, 2.2 * heading_err))
        if abs(heading_err) < math.pi / 2:
            speed = V_MAX * math.cos(heading_err) * min(1.0, dist / 1.5)
            if min_clear < 0.6:
                speed *= max(0.15, (min_clear - 0.25) / 0.35)
            cmd.linear.x = max(0.0, speed)
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
