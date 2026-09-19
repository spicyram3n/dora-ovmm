#!/usr/bin/env python3
"""Does every controller on the robot answer a request from this PC?

grasp_preflight.py's action checks are graph-level: wait_for_server returns as
soon as the server appears, which says nothing about whether a request would be
answered (docs/realrobot_grasp_test.md, problems 7 and 9). This sends a real
read-only round trip to each controller node and commands nothing.

The round trip is list_parameters, which every ROS node advertises. Only the two
JointTrajectoryControllers carry query_state; the gripper and the base run
Toyota's own controller types, which do not, so probing that service says
nothing about them.

A controller that is never discovered, or that takes a request without
answering, points at the DDS link rather than at the controller: see the
comment at the top of .devcontainer/cyclonedds_profile.xml.

    python3 realrobot/live/dds_check.py           # one pass
    python3 realrobot/live/dds_check.py 5         # five passes, fresh clients
"""
import sys
import time

import rclpy
from rcl_interfaces.srv import ListParameters

CONTROLLERS = ["arm_trajectory_controller", "head_trajectory_controller",
               "gripper_controller", "omni_base_controller"]
DISCOVER = 15.0
REPLY = 5.0


def probe(node, name):
    """Return (ok, message) for one read-only round trip to that node."""
    client = node.create_client(ListParameters, f"/{name}/list_parameters")
    started = time.monotonic()
    try:
        if not client.wait_for_service(timeout_sec=DISCOVER):
            return False, f"not discovered in {DISCOVER:.0f} s"
        discovered = time.monotonic() - started
        future = client.call_async(ListParameters.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=REPLY)
        if not future.done():
            return False, (f"discovered in {discovered:.1f} s, "
                           f"no reply to the request in {REPLY:.0f} s")
        return True, (f"discovered in {discovered:.1f} s, replied in "
                      f"{time.monotonic() - started - discovered:.2f} s")
    finally:
        node.destroy_client(client)


def main():
    passes = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    rclpy.init()
    failed = []
    for attempt in range(1, passes + 1):
        if passes > 1:
            print(f"--- pass {attempt}/{passes}")
        # A fresh node each pass, so every pass discovers from nothing.
        node = rclpy.create_node(f"dds_check_{attempt}")
        for name in CONTROLLERS:
            ok, message = probe(node, name)
            print(f"[{'ok  ' if ok else 'FAIL'}] {name}: {message}", flush=True)
            if not ok:
                failed.append(f"{name} (pass {attempt})")
        node.destroy_node()
    rclpy.shutdown()
    print("LINK OK" if not failed else "LINK NOT OK: " + ", ".join(failed))
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
