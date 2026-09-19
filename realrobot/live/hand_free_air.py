#!/usr/bin/env python3
"""Measure what the finger springs read while the EMPTY hand closes.

The real hand's springs deflect on nothing: 0.10 / 0.17 rad at command 0.76 and
0.53 / 0.60 at 0.30 (2026-09-19). core.grasping.pick judges contact as the rise
above this curve (pick.spring_rise), so the curve is a calibration of the hand,
independent of any object. It is indexed by the COMMAND: the fingers follow the
command, while the hand_motor_joint reading is the command minus the mean spring.

Steps the hand from open to --floor in 0.02 rad commands, --sweeps times, and
prints how well the sweeps agree. Nothing in the hand, arm stowed.

    HSR_REAL_ROBOT=1 python3 realrobot/live/hand_free_air.py            # measure, print
    HSR_REAL_ROBOT=1 python3 realrobot/live/hand_free_air.py --write    # and replace
                                                # core/grasping/free_air_springs.json

Needs the joint states from launch/realrobot/grasp_real.launch.py.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from core.grasping import pick

OUTPUT = Path(pick.__file__).parent / "free_air_springs.json"


def sweep(node, floor):
    pick.open_hand(node)
    rows, command = [], pick.OPEN_HAND
    while command - .02 >= floor - 1e-9:
        command -= .02
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["hand_motor_joint"]
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[command], time_from_start=Duration(nanosec=500000000))]
        pick.run_action(node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory", goal, 10.)
        q = pick.joint_positions(node)
        rows.append((command, q["hand_l_spring_proximal_joint"], q["hand_r_spring_proximal_joint"]))
        print(f"cmd {command:.2f}  L {rows[-1][1]:+.3f}  R {rows[-1][2]:+.3f}", flush=True)
    return np.array(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweeps", type=int, default=2)
    parser.add_argument("--floor", type=float, default=.30,
                        help="lowest command; 0.30 is a 39 mm pad gap, the fingers meet near 0.0")
    parser.add_argument("--write", action="store_true", help=f"replace {OUTPUT.name} with the mean of the sweeps")
    arguments = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("hand_free_air")
    try:
        sweeps = np.array([sweep(node, arguments.floor) for _ in range(arguments.sweeps)])
    finally:
        pick.open_hand(node)
    spread = np.ptp(sweeps[:, :, 1:], axis=0).max(axis=0)
    print(f"[FREE AIR] {arguments.sweeps} sweeps agree to L {spread[0]:.3f}, R {spread[1]:.3f} rad", flush=True)
    if arguments.write:
        mean = sweeps.mean(axis=0)
        OUTPUT.write_text(json.dumps([dict(command=round(float(c), 2), left=round(float(l), 4), right=round(float(r), 4))
                                      for c, l, r in mean], indent=1) + "\n")
        print(f"[FREE AIR] wrote {OUTPUT}", flush=True)


if __name__ == "__main__":
    main()
