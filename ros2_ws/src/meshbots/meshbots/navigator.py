"""Per-robot reactive navigator (runs inside the robot's namespace).

Potential-field local planner: attractive pull toward the current goal,
repulsive push from lidar returns, converted to unicycle (v, w) commands.
Teammates are avoided for free — they show up on lidar like any obstacle.
Stops when the goal goes stale so higher layers "hold" a robot simply by
not publishing; includes a wiggle-out stuck escape.

All gains live in config/navigation.yaml.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import LaserScan


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        p = self.declare_parameters('', [
            ('v_max', 0.8), ('w_max', 1.8), ('stop_dist', 0.35),
            ('goal_stale', 1.5), ('repulse_radius', 1.6),
            ('repulse_gain', 1.4), ('stuck_window', 5.0),
            ('stuck_dist', 0.08), ('wiggle_time', 1.6),
            ('cmd_vel_stamped', False)])   # ros2_control chassis want TwistStamped
        (self.v_max, self.w_max, self.stop_dist, self.goal_stale,
         self.repulse_r, self.repulse_k, self.stuck_window,
         self.stuck_dist, self.wiggle_t, self.stamped) = [x.value for x in p]

        self.pose = None
        self.goal = None
        self.goal_t = 0.0
        self.scan = None
        self.track = []            # (t, x, y) history for stuck detection
        self.wiggle_until = 0.0
        self.wiggle_sign = 1.0

        msg_type = TwistStamped if self.stamped else Twist
        self.pub_cmd = self.create_publisher(msg_type, 'cmd_vel', 10)
        self.create_subscription(PoseStamped, 'pose', self.on_pose, 20)
        self.create_subscription(PoseStamped, 'goal', self.on_goal, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 5)
        self.create_timer(0.1, self.tick)

    def send_cmd(self, twist):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist = twist
            self.pub_cmd.publish(msg)
        else:
            self.pub_cmd.publish(twist)

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
        self.send_cmd(Twist())
        self.track = []

    def tick(self):
        t = self.now()
        if (self.pose is None or self.goal is None
                or t - self.goal_t > self.goal_stale):
            self.stop()
            return

        x, y, yaw = self.pose
        gx, gy = self.goal
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist < self.stop_dist:
            self.stop()
            return

        # Stuck detection -> reverse-and-turn wiggle.
        self.track.append((t, x, y))
        self.track = [p for p in self.track if t - p[0] < self.stuck_window]
        if t < self.wiggle_until:
            cmd = Twist()
            cmd.linear.x = -0.25
            cmd.angular.z = self.wiggle_sign * 1.2
            self.send_cmd(cmd)
            return
        if len(self.track) > 20 and t - self.track[0][0] > self.stuck_window * 0.9:
            moved = math.hypot(x - self.track[0][1], y - self.track[0][2])
            if moved < self.stuck_dist:
                self.wiggle_until = t + self.wiggle_t
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
            near = valid & (r < self.repulse_r)
            if near.any():
                rr, aa = r[near], a[near]
                min_clear = float(rr.min())
                w = self.repulse_k * (self.repulse_r - rr) ** 2 / (rr + 0.05) / len(rr)
                fx += float(np.sum(-w * np.cos(aa)))
                fy += float(np.sum(-w * np.sin(aa)))

        heading_err = math.atan2(fy, fx)
        cmd = Twist()
        cmd.angular.z = max(-self.w_max, min(self.w_max, 2.2 * heading_err))
        if abs(heading_err) < math.pi / 2:
            speed = self.v_max * math.cos(heading_err) * min(1.0, dist / 1.5)
            if min_clear < 0.6:
                speed *= max(0.15, (min_clear - 0.25) / 0.35)
            cmd.linear.x = max(0.0, speed)
        self.send_cmd(cmd)


def main():
    rclpy.init()
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
