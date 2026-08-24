"""Per-robot mesh radio: MANET-style flooding with TTL and duplicate suppression.

Each robot owns one of these. Local applications (mapper, swarm) publish
application payloads on  <ns>/mesh/tx_app  and receive peer traffic on
<ns>/mesh/rx.  The radio wraps payloads into packets:

    {"src": "rover_1", "seq": 42, "ttl": 4, "path": ["rover_1"],
     "type": "BEACON", "data": {...}}

and puts them "on the air" (<ns>/mesh/air_tx). The radio_channel node models
RF propagation and delivers packets only to radios within range
(<ns>/mesh/air_rx). A radio that hears a packet it has not seen before
delivers it to its local apps AND re-broadcasts it with ttl-1 — this is the
relay behaviour that turns every rover into a network node. Packets reach
out-of-range robots only by hopping through intermediates; the "path" field
records the actual route taken.
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MeshRadio(Node):
    def __init__(self):
        super().__init__('mesh_radio')
        self.declare_parameter('robot', 'rover_1')
        self.declare_parameter('ttl', 4)
        self.me = self.get_parameter('robot').value
        self.default_ttl = int(self.get_parameter('ttl').value)

        self.seq = 0
        self.seen = {}          # (src, seq) -> stamp sec
        self.stats = {'tx': 0, 'rx': 0, 'relayed': 0, 'dup_dropped': 0}

        ns = f'/{self.me}/mesh'
        self.pub_air = self.create_publisher(String, f'{ns}/air_tx', 50)
        self.pub_local = self.create_publisher(String, f'{ns}/rx', 50)
        self.create_subscription(String, f'{ns}/tx_app', self.on_app_tx, 50)
        self.create_subscription(String, f'{ns}/air_rx', self.on_air_rx, 50)
        self.pub_stats = self.create_publisher(String, f'{ns}/stats', 5)
        self.create_timer(2.0, self.publish_stats)
        self.create_timer(10.0, self.gc_seen)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_app_tx(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn('dropping malformed app payload')
            return
        self.seq += 1
        pkt = {
            'src': self.me,
            'seq': self.seq,
            'ttl': int(payload.get('ttl', self.default_ttl)),
            'path': [self.me],
            'type': payload.get('type', 'DATA'),
            'data': payload.get('data', {}),
        }
        self.seen[(self.me, self.seq)] = self.now_sec()
        self.stats['tx'] += 1
        self.pub_air.publish(String(data=json.dumps(pkt)))

    def on_air_rx(self, msg: String):
        try:
            pkt = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        src, seq = pkt.get('src'), pkt.get('seq')
        if src == self.me:
            return
        if (src, seq) in self.seen:
            self.stats['dup_dropped'] += 1
            return
        self.seen[(src, seq)] = self.now_sec()
        self.stats['rx'] += 1
        # Deliver to local applications.
        self.pub_local.publish(String(data=json.dumps(pkt)))
        # Relay: flood onward so out-of-range robots can hear it through us.
        if pkt.get('ttl', 0) > 1:
            relay = dict(pkt)
            relay['ttl'] = pkt['ttl'] - 1
            relay['path'] = pkt.get('path', []) + [self.me]
            self.stats['relayed'] += 1
            self.pub_air.publish(String(data=json.dumps(relay)))

    def publish_stats(self):
        self.pub_stats.publish(String(data=json.dumps(self.stats)))

    def gc_seen(self):
        cutoff = self.now_sec() - 30.0
        self.seen = {k: t for k, t in self.seen.items() if t > cutoff}


def main():
    rclpy.init()
    node = MeshRadio()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
