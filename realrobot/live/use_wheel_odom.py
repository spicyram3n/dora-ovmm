#!/usr/bin/env python3
"""Put the robot's odometry_switcher, and so Nav2 and AMCL, on wheel odometry.

The robot boots with /switched_odom following /laser_odom, Toyota's scan-matching
odometry. A scan matcher cannot see forward motion along a corridor, where every
scan looks like the last, so it under-reports it; AMCL cannot see along a corridor
either, so nothing corrects it. Twice on 2026-09-19 AMCL ended up behind the robot
along its heading with the heading itself right -- by 2.2 m, then by 4.4 m, which
drove the base past the kitchen into a hallway that was never mapped. Wheel
odometry errs the same everywhere, corridor or not.

The switcher keeps the pose continuous across the change, and falls back to
laser_odom whenever the robot's stack restarts, so this runs with every Nav2 launch
(launch/realrobot/nav2_real.launch.py) and before realrobot/live/localize.py.

    python3 realrobot/live/use_wheel_odom.py              # wheel odometry
    python3 realrobot/live/use_wheel_odom.py laser_odom   # back, to compare the two
"""
import argparse
import sys

import rclpy
from std_msgs.msg import String
from tmc_navigation_msgs.srv import OdometrySwitch


def switch(node, source="wheel_odom", attempts=3, timeout=5.0):
    """True once the switcher confirms `source`, False if it refuses or never answers."""
    client = node.create_client(OdometrySwitch, "/odometry_switch")
    if not client.wait_for_service(timeout_sec=30.0):
        print("[ODOM] /odometry_switch is not available; is the robot's stack up?", flush=True)
        return False
    request = OdometrySwitch.Request(odom_type=String(data=source))
    # Resent when unanswered: this link loses a fresh client's first answer, which is
    # how the same call hung from the command line. Asking twice is harmless.
    for attempt in range(1, attempts + 1):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if future.done():
            return bool(future.result().is_success)
        print(f"[ODOM] no answer in {timeout:.0f} s (attempt {attempt}/{attempts})", flush=True)
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", nargs="?", default="wheel_odom", choices=["wheel_odom", "laser_odom"])
    source = parser.parse_args().source
    rclpy.init()
    node = rclpy.create_node("use_wheel_odom")
    try:
        done = switch(node, source)
        if done:
            print(f"[ODOM] Nav2 and AMCL are on {source.replace('_odom', '')} odometry", flush=True)
        else:
            print(f"[ODOM] the switch to {source} FAILED; on laser odometry expect drift in corridors", flush=True)
        return 0 if done else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
