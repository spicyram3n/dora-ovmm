"""Omit unchanged OctoMaps only from scene diffs, preserving full snapshots."""
# MoveIt Humble PlanningScene::setPlanningSceneDiffMsg ignores an empty map;
# setPlanningSceneMsg replaces it. Preserve that distinction here.
# https://github.com/moveit/moveit2/blob/humble/moveit_core/planning_scene/src/planning_scene.cpp
from octomap_msgs.msg import OctomapWithPose
from rclpy.serialization import serialize_message


def compact(solution):
    previous = serialize_message(solution.start_scene.world.octomap)
    for step in solution.sub_trajectory:
        scene = step.scene_diff
        encoded = serialize_message(scene.world.octomap)
        if scene.is_diff and scene.world.octomap.octomap.data and encoded == previous:
            scene.world.octomap = OctomapWithPose()
        elif scene.world.octomap.octomap.data or not scene.is_diff:
            previous = encoded
    return solution
