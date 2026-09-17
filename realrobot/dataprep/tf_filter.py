"""Republish /tf_bag as /tf without the bag's map->odom.

The robot's own localizer was running while bags/playable_bag was recorded, so
its /tf carries a map->odom that wanders by tens of metres. slam_toolbox
publishes its own map->odom; two publishers of one edge break the tree, so
the bag is played with /tf remapped to /tf_bag and this node passes the rest.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TfFilter(Node):
    def __init__(self):
        super().__init__("tf_filter")
        self.publisher = self.create_publisher(TFMessage, "/tf", 100)
        self.create_subscription(TFMessage, "/tf_bag", self.relay, 100)
        self.dropped = 0

    def relay(self, message):
        kept = [t for t in message.transforms if t.header.frame_id != "map"]
        self.dropped += len(message.transforms) - len(kept)
        if kept:
            message.transforms = kept
            self.publisher.publish(message)


def main():
    rclpy.init()
    node = TfFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(f"tf_filter dropped {node.dropped} map->* transforms", flush=True)
        node.destroy_node()


if __name__ == "__main__":
    main()
