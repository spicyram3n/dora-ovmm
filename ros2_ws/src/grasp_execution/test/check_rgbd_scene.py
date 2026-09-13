"""Read-only live check: RGB-D subscriptions, populated map, collision-aware IK.

Run inside the sourced container with move_group and simulation running.
No motion is commanded. Candidate poses intentionally include points in
the table to demonstrate rejection by the octomap.
"""
import json
import time

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene, GetPositionIK, GetStateValidity


def main():
    rclpy.init()
    node = Node("check_rgbd_scene")

    def call(kind, name, request):
        client = node.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f"Missing service {name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done():
            raise RuntimeError(f"Timed out: {name}")
        node.destroy_client(client)
        return future.result()

    try:
        request = GetPlanningScene.Request()
        request.components.components = 2 | 32 | 128
        deadline = time.monotonic() + 30
        while True:
            scene = call(GetPlanningScene, "/get_planning_scene", request).scene
            if scene.world.octomap.octomap.data or time.monotonic() > deadline:
                break
            rclpy.spin_once(node, timeout_sec=1)
        subscriptions = node.get_subscriber_names_and_types_by_node("move_group", "/")
        assert not any("sensor_msgs/msg/PointCloud2" in types for _, types in subscriptions), subscriptions
        assert any("sensor_msgs/msg/Image" in types for _, types in subscriptions), subscriptions
        assert any("sensor_msgs/msg/CameraInfo" in types for _, types in subscriptions), subscriptions
        assert scene.world.octomap.octomap.data, "Depth image octomap is empty"
        print(json.dumps({"subscriptions": subscriptions,
                          "octomap_bytes": len(scene.world.octomap.octomap.data),
                          "octomap_frame": scene.world.octomap.header.frame_id}, indent=2), flush=True)

        # Probe in odom near this test table. Check the geometric solution's
        # contacts first so a generic IK failure cannot pass as collision proof.
        found = False
        for x, y, z in [(x, y, z) for y in (0.0, -0.25, 0.25)
                        for x, z in [(0.5, 1.06), (0.5, 1.03), (0.6, 1.03), (0.5, 1.1), (0.6, 1.1)]]:
            req = GetPositionIK.Request()
            ik = req.ik_request
            ik.group_name = "whole_body"
            ik.ik_link_name = "hand_palm_link"
            ik.robot_state = scene.robot_state
            ik.pose_stamped.header.frame_id = "odom"
            ik.pose_stamped.pose.position.x = x
            ik.pose_stamped.pose.position.y = y
            ik.pose_stamped.pose.position.z = z
            ik.pose_stamped.pose.orientation.y = 0.7071067811865476
            ik.pose_stamped.pose.orientation.w = 0.7071067811865476
            ik.timeout.sec = 1
            ik.avoid_collisions = False
            unchecked = call(GetPositionIK, "/compute_ik", req)
            if unchecked.error_code.val != 1:
                continue
            valid_req = GetStateValidity.Request()
            valid_req.robot_state = unchecked.solution
            valid_req.group_name = "whole_body"
            validity = call(GetStateValidity, "/check_state_validity", valid_req)
            contacts = [(c.contact_body_1, c.contact_body_2) for c in validity.contacts]
            if not any("<octomap>" in pair for pair in contacts):
                continue
            ik.avoid_collisions = True
            ik.robot_state = unchecked.solution
            checked = call(GetPositionIK, "/compute_ik", req)
            if checked.error_code.val == 1:
                valid_req.robot_state = checked.solution
                assert call(GetStateValidity, "/check_state_validity", valid_req).valid
            print(json.dumps({"probe": [x, y, z], "unchecked_ik": unchecked.error_code.val,
                              "unchecked_contacts": contacts, "checked_ik": checked.error_code.val}), flush=True)
            found = True
            break
        assert found, "No octomap-contact IK probe found; collision rejection remains unverified"
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
