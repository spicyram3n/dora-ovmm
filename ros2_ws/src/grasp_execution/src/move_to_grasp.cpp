// Tries a list of candidate grasp poses, best-scored first, until MoveIt
// reaches one. Two reasons a given candidate can fail:
//  - GraspGenX scores grasps on geometry alone, with no idea which
//    orientations HSR's arm can actually realize.
//  - GraspGenX poses are the final, closed-gripper contact pose. By
//    definition the gripper is touching or enclosing the object there, and
//    a collision-aware planner will never plan straight into that. So this
//    moves to a "pregrasp" pose instead: the same orientation, offset back
//    along the grasp's own approach (+Z) axis. Actually closing the last
//    bit onto the object is a separate step (short Cartesian move,
//    collision checking relaxed) not done here.
//
// The right standoff distance isn't a fixed constant. It depends on how
// much clearance the current scene needs (sensors_xtion.yaml pads every
// perceived point by 10cm before it counts as an obstacle). So for each
// grasp candidate, this tries kStandoffs from closest to farthest and takes
// the first that's both reachable and collision-free: the tightest
// approach that actually works, not just any that does.
//
// Reads parallel arrays (xs, ys, zs, qxs, qys, qzs, qws, scores) plus
// frame_id and group_name as ROS parameters. See move_to_grasp.launch.py,
// which builds these from a grasp YAML saved by core/run_pipeline.py.
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>

#include <rclcpp/rclcpp.hpp>

namespace {

const std::vector<double> kStandoffs = {0.05, 0.07, 0.10, 0.12, 0.15, 0.20, 0.25};

// Third column of the rotation matrix for quaternion (x, y, z, w): the
// world-frame direction of the pose's own local +Z axis.
geometry_msgs::msg::Point ApproachAxis(const geometry_msgs::msg::Quaternion& q) {
  geometry_msgs::msg::Point axis;
  axis.x = 2.0 * (q.x * q.z + q.w * q.y);
  axis.y = 2.0 * (q.y * q.z - q.w * q.x);
  axis.z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  return axis;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
      "move_to_grasp", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  std::string frame_id = node->get_parameter("frame_id").as_string();
  std::string group_name = node->get_parameter("group_name").as_string();
  auto xs = node->get_parameter("xs").as_double_array();
  auto ys = node->get_parameter("ys").as_double_array();
  auto zs = node->get_parameter("zs").as_double_array();
  auto qxs = node->get_parameter("qxs").as_double_array();
  auto qys = node->get_parameter("qys").as_double_array();
  auto qzs = node->get_parameter("qzs").as_double_array();
  auto qws = node->get_parameter("qws").as_double_array();
  auto scores = node->get_parameter("scores").as_double_array();

  moveit::planning_interface::MoveGroupInterface group(node, group_name);
  group.setMaxVelocityScalingFactor(0.3);
  group.setMaxAccelerationScalingFactor(0.3);

  for (size_t i = 0; i < xs.size(); ++i) {
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = frame_id;
    target.pose.orientation.x = qxs[i];
    target.pose.orientation.y = qys[i];
    target.pose.orientation.z = qzs[i];
    target.pose.orientation.w = qws[i];
    auto approach = ApproachAxis(target.pose.orientation);

    for (double standoff : kStandoffs) {
      target.pose.position.x = xs[i] - standoff * approach.x;
      target.pose.position.y = ys[i] - standoff * approach.y;
      target.pose.position.z = zs[i] - standoff * approach.z;
      group.setPoseTarget(target);

      RCLCPP_INFO(logger, "grasp %zu/%zu (score=%.2f): planning '%s' to a %.2fm pregrasp in %s...",
                  i + 1, xs.size(), scores[i], group_name.c_str(), standoff, frame_id.c_str());
      auto result = group.move();

      if (result) {
        RCLCPP_INFO(logger, "reached grasp %zu/%zu at a %.2fm standoff", i + 1, xs.size(), standoff);
        rclcpp::shutdown();
        return EXIT_SUCCESS;
      }
      RCLCPP_WARN(logger, "grasp %zu/%zu at %.2fm failed (error code %d)", i + 1, xs.size(), standoff, result.val);
    }
  }

  RCLCPP_ERROR(logger, "none of the %zu candidate grasps were reachable at any standoff", xs.size());
  rclcpp::shutdown();
  return EXIT_FAILURE;
}
