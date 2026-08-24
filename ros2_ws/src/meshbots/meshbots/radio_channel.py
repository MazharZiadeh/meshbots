"""RF propagation simulator ("the air") — ISAC edition.

The ONLY non-per-robot node, and it deliberately contains no coordination
logic: it models physics that a real deployment gets from actual radios.

For every transmitted packet and every potential receiver it evaluates the
shared path-loss model (log-distance + obstacle penetration from
rf_model.py, matching the arena geometry), draws a noisy RSSI, and delivers
the packet with probability PDR(RSSI). Delivered packets are stamped with
the RSSI the receiving radio measured — that stamp is the raw material for
the whole communication-as-sensing layer. Robots never see ground truth;
they only see what their radio heard.

Also simulated here: cheap transmit-only RF tags on the delivery pads
(mission infrastructure, not a coordinator). Each tag chirps its surveyed
position once a second; rovers in range receive it like any packet, with
RSSI, giving the team absolute anchors that kill common-mode drift.

Publishes the live link graph as RViz markers, colored by link quality.
"""
import json
import math
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from . import rf_model


class RadioChannel(Node):
    def __init__(self):
        super().__init__('radio_channel')
        self.declare_parameter('robots', ['rover_1', 'rover_2', 'rover_3'])
        self.declare_parameter('spawns', [0.0, 0.0, 0.0] * 3)  # x,y,yaw each
        self.declare_parameter('anchors', [0.0])               # flat x,y pads

        self.robots = list(self.get_parameter('robots').value)
        flat = list(self.get_parameter('spawns').value)
        self.spawn = {r: flat[3 * i:3 * i + 3] for i, r in enumerate(self.robots)}
        aflat = list(self.get_parameter('anchors').value)
        self.anchors = {f'pad_{i}': (aflat[2 * i], aflat[2 * i + 1])
                        for i in range(len(aflat) // 2)}
        self.anchor_seq = 0

        # Ground-truth world position per robot (this node IS the physics).
        self.pos = {r: (self.spawn[r][0], self.spawn[r][1]) for r in self.robots}

        self.pub_rx = {}
        for r in self.robots:
            self.pub_rx[r] = self.create_publisher(String, f'/{r}/mesh/air_rx', 50)
            self.create_subscription(
                String, f'/{r}/mesh/air_tx',
                lambda m, sender=r: self.on_tx(sender, m), 50)
            self.create_subscription(
                Odometry, f'/{r}/odom',
                lambda m, rr=r: self.on_odom(rr, m), 20)

        self.pub_links = self.create_publisher(MarkerArray, '/mesh/links', 5)
        self.create_timer(0.5, self.publish_links)
        self.create_timer(1.0, self.anchor_chirps)

    def on_odom(self, robot, msg: Odometry):
        sx, sy, syaw = self.spawn[robot]
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        c, s = math.cos(syaw), math.sin(syaw)
        self.pos[robot] = (sx + c * ox - s * oy, sy + s * ox + c * oy)

    def sample_link(self, ax, ay, bx, by):
        """Draw one (rssi, delivered?) sample for a link."""
        rssi = rf_model.link_rssi(ax, ay, bx, by,
                                  noise=random.gauss(0.0, rf_model.SIGMA_DB))
        return rssi, random.random() < rf_model.pdr(rssi)

    def on_tx(self, sender, msg: String):
        try:
            pkt = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        ax, ay = self.pos[sender]
        for r in self.robots:
            if r == sender:
                continue
            bx, by = self.pos[r]
            rssi, delivered = self.sample_link(ax, ay, bx, by)
            if not delivered:
                continue
            out = dict(pkt)
            out['rf'] = {'rssi': round(rssi, 1)}   # what THIS radio measured
            self.pub_rx[r].publish(String(data=json.dumps(out)))

    def anchor_chirps(self):
        """Delivery-pad RF tags: transmit-only position chirps."""
        self.anchor_seq += 1
        for name, (px, py) in self.anchors.items():
            for r in self.robots:
                bx, by = self.pos[r]
                rssi, delivered = self.sample_link(px, py, bx, by)
                if not delivered:
                    continue
                pkt = {'src': name, 'seq': self.anchor_seq, 'ttl': 1,
                       'path': [name], 'type': 'ANCHOR',
                       'data': {'p': [px, py]},
                       'rf': {'rssi': round(rssi, 1)}}
                self.pub_rx[r].publish(String(data=json.dumps(pkt)))

    def publish_links(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mid = 0
        for i, a in enumerate(self.robots):
            for b in self.robots[i + 1:]:
                (axp, ayp), (bxp, byp) = self.pos[a], self.pos[b]
                mean_rssi = rf_model.link_rssi(axp, ayp, bxp, byp)
                quality = rf_model.pdr(mean_rssi)
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = stamp
                m.ns = 'mesh_links'
                m.id = mid
                mid += 1
                m.type = Marker.LINE_LIST
                if quality < 0.05:
                    m.action = Marker.DELETE
                else:
                    m.action = Marker.ADD
                    m.scale.x = 0.06 + 0.06 * quality
                    # green = solid link, red = dying link
                    m.color.r = float(1.0 - quality)
                    m.color.g = float(quality)
                    m.color.b = 0.15
                    m.color.a = 0.9
                    m.points = [Point(x=axp, y=ayp, z=0.5),
                                Point(x=bxp, y=byp, z=0.5)]
                arr.markers.append(m)
        self.pub_links.publish(arr)


def main():
    rclpy.init()
    node = RadioChannel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
