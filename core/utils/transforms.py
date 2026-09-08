"""Conversions between ROS geometry messages and (4, 4) matrices."""

import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation


def matrix_from_transform(transform):
    """geometry_msgs/Transform -> (4, 4)."""
    rotation, translation = (transform.rotation, transform.translation)
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(
        [rotation.x, rotation.y, rotation.z, rotation.w]
    ).as_matrix()
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def pose_from_matrix(matrix):
    """(4, 4) -> geometry_msgs/Pose."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = matrix[:3, 3]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        Rotation.from_matrix(matrix[:3, :3]).as_quat()
    )
    return pose


def translation_matrix(x, y, z=0.0):
    """(4, 4) that shifts by (x, y, z) and does not rotate."""
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return matrix


def yaw_from_quaternion(quaternion):
    """The single angle a planar pose has: yaw = 2 * atan2(z, w)."""
    return 2.0 * np.arctan2(quaternion.z, quaternion.w)
