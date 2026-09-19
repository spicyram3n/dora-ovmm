"""Republish /tf_bag as /tf without the bag's map->odom.

The robot's own localizer was running while bags/playable_bag was recorded, so
its /tf carries a map->odom that wanders by tens of metres. slam_toolbox
publishes its own map->odom; two publishers of one edge break the tree, so
the bag is played with /tf remapped to /tf_bag and this node passes the rest.

With --wheel-odom it also rebuilds the odometry SLAM sees. The robot's
odom->base_footprint is laser scan-matching odometry, which cannot see motion
along a corridor, and neither can SLAM's own scan matching, so the corridor by
the lab's office was mapped about 2.4 m short. The bag also carries
odom->base_footprint_wheel; see wheel_odometry for how the two are combined.
"""

import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def _yaw(rotation):
    return 2 * math.atan2(rotation.z, rotation.w)      # planar: the base only yaws


def wheel_odometry(transforms, state):
    """`transforms` with odom->base_footprint rebuilt: distance from the wheels, heading
    from the scan matcher. `state` is a dict this keeps between calls.

    Each odometry is good at what the other is bad at (lab bag, 2026-09-19). The scan
    matcher loses 14-40% of the distance along a corridor but holds heading; the wheels
    are within 1-2% on distance but their yaw drifts 70 deg over the run, up to 8 deg in
    3 s. So every step the wheels report is taken in the wheels' own frame, where slip
    in yaw does not reach it, and laid down along the scan matcher's heading."""
    kept = []
    for t in transforms:
        edge = (t.header.frame_id, t.child_frame_id)
        if edge == ("odom", "base_footprint"):
            state["heading"] = _yaw(t.transform.rotation)
        elif edge == ("odom", "base_footprint_wheel"):
            x, y, yaw = t.transform.translation.x, t.transform.translation.y, _yaw(t.transform.rotation)
            heading = state.get("heading", yaw)
            if "wheel" in state:
                last_x, last_y, last_yaw = state["wheel"]
                forward = math.cos(last_yaw) * (x - last_x) + math.sin(last_yaw) * (y - last_y)
                left = -math.sin(last_yaw) * (x - last_x) + math.cos(last_yaw) * (y - last_y)
                state["x"] += math.cos(state["laid"]) * forward - math.sin(state["laid"]) * left
                state["y"] += math.sin(state["laid"]) * forward + math.cos(state["laid"]) * left
            else:
                state["x"], state["y"] = x, y
            state["wheel"], state["laid"] = (x, y, yaw), heading
            t.child_frame_id = "base_footprint"
            t.transform.translation.x, t.transform.translation.y = state["x"], state["y"]
            t.transform.rotation.z, t.transform.rotation.w = math.sin(heading / 2), math.cos(heading / 2)
            kept.append(t)
        else:
            kept.append(t)
    return kept


class TfFilter(Node):
    def __init__(self):
        super().__init__("tf_filter")
        self.publisher = self.create_publisher(TFMessage, "/tf", 100)
        self.create_subscription(TFMessage, "/tf_bag", self.relay, 100)
        self.dropped = 0
        self.wheel = "--wheel-odom" in sys.argv
        self.odometry = {}

    def relay(self, message):
        kept = [t for t in message.transforms if t.header.frame_id != "map"]
        self.dropped += len(message.transforms) - len(kept)
        if self.wheel:
            kept = wheel_odometry(kept, self.odometry)
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
