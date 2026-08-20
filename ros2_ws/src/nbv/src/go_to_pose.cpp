// Send one nav2 goal and wait for the robot to get there.
//
//   ros2 run nbv go_to_pose --ros-args -p x:=0.0 -p y:=-5.95
//   ros2 run nbv go_to_pose --ros-args -p x:=2.86 -p y:=-2.86 -p yaw:=-0.785
//
// Coordinates are absolute in the map frame, not relative to the robot.
// `ros2 run nbv where_am_i.py` prints where it currently is.
//
// Exits 0 when it arrives, 1 otherwise, so it can be used in a shell script.

#include <chrono>
#include <cmath>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("go_to_pose");
  auto log = node->get_logger();

  const double x = node->declare_parameter<double>("x", 0.0);
  const double y = node->declare_parameter<double>("y", 0.0);
  const double yaw = node->declare_parameter<double>("yaw", 0.0);
  const auto frame = node->declare_parameter<std::string>("frame_id", "map");

  auto client = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");
  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(log, "navigate_to_pose not available, is nav2 running?");
    return 1;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = frame;
  goal.pose.header.stamp = node->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  // The base is planar, so the goal rotation is about +Z alone and the
  // quaternion collapses to these two terms.
  goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

  RCLCPP_INFO(log, "going to (%.2f, %.2f) yaw %.2f in %s", x, y, yaw, frame.c_str());

  auto sent = client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, sent) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(log, "could not send the goal");
    return 1;
  }
  auto handle = sent.get();
  if (!handle) {
    RCLCPP_ERROR(log, "nav2 rejected the goal, is it reachable and on the map?");
    return 1;
  }

  auto result = client->async_get_result(handle);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(log, "gave up waiting for the result");
    return 1;
  }

  // nav2 reports the reason only in the controller_server log, so on failure
  // look there rather than at the empty result message.
  const bool arrived = result.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  RCLCPP_INFO(log, arrived ? "arrived" : "did not arrive, check the controller_server log");

  rclcpp::shutdown();
  return arrived ? 0 : 1;
}
