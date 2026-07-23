"""Cykliczny rebroadcast /tf_static — wsparcie klientow websocket (Foxglove).

/tf_static jest latched (transient_local) i publikowany JEDNORAZOWO (RSP przy
starcie). foxglove_bridge nie zawsze dostarcza latched sample klientom
dolaczajacym/reconnectujacym pozniej — w Studio "rozsypuje" sie model robota
(czlony na spawach fixed bez transformu). Ten wezel agreguje wszystkie
transformy z /tf_static (po child_frame_id) i rozglasza komplet co
`period` sekund: kazdy klient ma statyki najpozniej po jednym okresie.

Dla tf2 to no-op (statyki sa bezczasowe, ponowny set identycznej wartosci
nic nie zmienia); koszt ~1 mala wiadomosc na okres.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from tf2_msgs.msg import TFMessage

QOS_LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
)


class TfStaticRepublisher(Node):
    def __init__(self) -> None:
        super().__init__('tf_static_republisher')
        self.declare_parameter('period', 3.0)
        self._transforms = {}  # child_frame_id -> TransformStamped
        self._pub = self.create_publisher(TFMessage, '/tf_static', QOS_LATCHED)
        self._sub = self.create_subscription(
            TFMessage, '/tf_static', self._on_static, QOS_LATCHED)
        period = float(self.get_parameter('period').value)
        self.create_timer(period, self._rebroadcast)
        self.get_logger().info(f'tf_static rebroadcast co {period:.1f} s')

    def _on_static(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self._transforms[t.child_frame_id] = t

    def _rebroadcast(self) -> None:
        if not self._transforms:
            return
        out = TFMessage()
        out.transforms = list(self._transforms.values())
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfStaticRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
