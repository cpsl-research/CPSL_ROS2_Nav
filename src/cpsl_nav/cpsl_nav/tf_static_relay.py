"""
Copy /tf_static onto one robot's own static tf topic, for Nav2.

Stock nav2_bringup remaps /tf and /tf_static to the relative ``tf`` and
``tf_static``, so in a namespace every Nav2 node reads ``/<ns>/tf`` and
``/<ns>/tf_static``. ``/<ns>/tf`` is where a Create 3's ``odom -> base_link``
and the mapper's ``map -> odom`` already are, but sensor mounts
(``base_link -> laser``) are usually on the global /tf_static, which the
mapper reads and Nav2, after that remap, would not. This makes Nav2's tree the
mapper's: everything on ``/<ns>/tf`` plus every static transform.

One direction only, and refused when the output is /tf_static itself, so it
can never feed itself. Run by console_nav.launch.py when a namespace is set::

    ros2 run cpsl_nav tf_static_relay /<ns>/tf_static
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


def _latched(depth):
    return QoSProfile(depth=depth, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=QoSReliabilityPolicy.RELIABLE)


class Relay(Node):
    """Republish every static transform seen on /tf_static, latched, on `out_topic`."""

    def __init__(self, out_topic):
        super().__init__('tf_static_relay')
        self._have = {}
        self._pub = self.create_publisher(TFMessage, out_topic, _latched(1))
        self.create_subscription(TFMessage, '/tf_static', self._on, _latched(100))
        self.get_logger().info(f'relaying /tf_static -> {out_topic}')

    def _on(self, msg):
        # One message holding all of them, as StaticTransformBroadcaster does,
        # so a late subscriber gets the lot.
        changed = False
        for t in msg.transforms:
            key = (t.header.frame_id, t.child_frame_id)
            if self._have.get(key) != t:
                self._have[key] = t
                changed = True
        if changed:
            self._pub.publish(TFMessage(transforms=list(self._have.values())))


def main(argv=None):
    args = rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)
    out = args[1] if len(args) > 1 else ''
    if not out.startswith('/') or out == '/tf_static':
        print(f'tf_static_relay: refusing output topic {out!r}', file=sys.stderr)
        return 2
    rclpy.init(args=argv)
    node = Relay(out)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
