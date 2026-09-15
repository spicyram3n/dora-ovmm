"""Target-centred TSDF fusion for active grasping, after ETH's active_grasp.

Ported from https://github.com/ethz-asl/vgn (src/vgn/perception.py) and
https://github.com/ethz-asl/active_grasp (src/active_grasp/policy.py, bbox.py).
Their frames (ROS 1 tf, robot_helpers.Transform) become plain (4, 4) numpy
matrices here, so nothing in this package needs ROS.
"""
