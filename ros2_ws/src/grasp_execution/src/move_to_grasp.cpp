#include <algorithm>
#include <chrono>
#include <limits>
#include <numeric>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tmc_control_msgs/action/gripper_apply_effort.hpp>

namespace {

// The grasp action closes until the fingers stall on the object. Negative
// effort closes, positive opens, and the magnitude must exceed the drive's
// minimum (~0.011 Nm) or nothing moves.
constexpr double kGraspEffortNm = -0.3;
// Distal fingers mimic -motor - 0.087 and have a -1.24 lower bound.
// The motor's own 1.24 limit therefore cannot be used as an opening goal.
constexpr double kOpenGripperRadians = 1.10;
constexpr double kLiftMeters = 0.10;
constexpr double kPregraspOffsetMeters = 0.08;

// Register only the target after reaching pregrasp. MoveIt's depth updater
// filters modeled world objects, so furniture remains in the existing map.
// A named object makes finger contact permissions independent of the octomap.
bool SeparateTargetAtPregrasp(const rclcpp::Node::SharedPtr& node,
                             const std::string& frame, double x, double y,
                             double lo, double hi, double radius) {
  if (frame != "odom" || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(lo) || !std::isfinite(hi) || !std::isfinite(radius) ||
      radius < 0.015 || radius > 0.06 || hi - lo < 0.04 || hi - lo > 0.4) return false;
  auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
                                   moveit_msgs::msg::PlanningSceneComponents::OCTOMAP;
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) return false;
  auto scene = future.get()->scene;
  collision_detection::AllowedCollisionMatrix acm(scene.allowed_collision_matrix);
  acm.setEntry("grasp_target", false);
  for (const auto* link : {"hand_l_distal_link", "hand_r_distal_link",
                           "hand_l_spring_proximal_link", "hand_r_spring_proximal_link"})
    acm.setEntry("grasp_target", link, true);
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame;
  object.id = "grasp_target";
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive cylinder;
  cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder.dimensions = {hi - lo, radius};
  geometry_msgs::msg::Pose pose;
  pose.position.x = x; pose.position.y = y; pose.position.z = (lo + hi) / 2;
  pose.orientation.w = 1;
  object.primitives.push_back(cylinder);
  object.primitive_poses.push_back(pose);
  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  diff.world.collision_objects.push_back(object);
  acm.getMessage(diff.allowed_collision_matrix);
  if (!moveit::planning_interface::PlanningSceneInterface().applyPlanningScene(diff)) return false;
  auto exclusion = node->create_publisher<moveit_msgs::msg::CollisionObject>("/octomap_target_exclusion", 1);
  // Wait for native RGB-D filtering, without clearing or replacing the map.
  // Check the can's side-contact band, excluding its table contact at the base.
  // Residual voxels elsewhere still participate in the subsequent path checks.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    exclusion->publish(object);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) return false;
    const auto map = future.get()->scene.world.octomap;
    if (map.header.frame_id != frame) return false;
    std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::msgToMap(map.octomap));
    const auto* tree = dynamic_cast<const octomap::OcTree*>(abstract.get());
    if (!tree) return false;
    size_t target_cells = 0, other_cells = 0;
    const double padded_radius = radius + tree->getResolution();
    for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
      if (!tree->isNodeOccupied(*it)) continue;
      const double dx = it.getX() - x, dy = it.getY() - y;
      if (dx * dx + dy * dy <= padded_radius * padded_radius &&
          it.getZ() > lo + 0.025 && it.getZ() < hi - 0.015) ++target_cells;
      else ++other_cells;
    }
    if (target_cells == 0 && other_cells > 0) {
      RCLCPP_INFO(node->get_logger(), "target contact-band voxels cleared by RGB-D filtering; %zu other occupied cells retained", other_cells);
      return true;
    }
  }
  RCLCPP_ERROR(node->get_logger(), "target filtering did not finish; stopping at pregrasp");
  return false;
}

// Keep every robot link collision-checked against the RGB-D map, including
// entries left behind by an older executor. Preserve SRDF self-collision pairs.
bool PrepareCollisionScene(const rclcpp::Node::SharedPtr& node,
                           const moveit::core::RobotModelConstPtr& model) {
  auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  if (!client->wait_for_service(std::chrono::seconds(10))) return false;
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
                                   moveit_msgs::msg::PlanningSceneComponents::OCTOMAP |
                                   moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                   moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) return false;
  auto scene = future.get()->scene;
  for (const auto& object : scene.world.collision_objects) {
    if (object.id == "grasp_target") {
      RCLCPP_ERROR(node->get_logger(), "previous grasp_target still excludes depth; start a fresh planning scene and recapture before another pregrasp");
      return false;
    }
  }
  for (const auto& attached : scene.robot_state.attached_collision_objects) {
    if (attached.object.id == "grasp_target") return false;
  }
  if (scene.world.octomap.octomap.data.empty()) {
    RCLCPP_ERROR(node->get_logger(), "RGB-D octomap is empty; refusing to plan without obstacles");
    return false;
  }

  collision_detection::AllowedCollisionMatrix acm(scene.allowed_collision_matrix);
  for (const auto& link : model->getLinkModelNames()) {
    acm.setEntry("<octomap>", link, false);
  }

  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  acm.getMessage(diff.allowed_collision_matrix);
  return moveit::planning_interface::PlanningSceneInterface().applyPlanningScene(diff);
}

bool SetCollisionAwareIK(const rclcpp::Node::SharedPtr& node,
                         moveit::planning_interface::MoveGroupInterface& group,
                         const geometry_msgs::msg::PoseStamped& pose) {
  auto client = node->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");
  if (!client->wait_for_service(std::chrono::seconds(10))) return false;
  auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  auto& ik = request->ik_request;
  ik.group_name = group.getName();
  ik.ik_link_name = "hand_palm_link";
  ik.pose_stamped = pose;
  ik.avoid_collisions = true;
  ik.constraints = group.getPathConstraints();
  ik.timeout.sec = 1;
  ik.robot_state.is_diff = true;
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) return false;
  auto response = future.get();
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_WARN(node->get_logger(), "collision-aware IK rejected pose with error %d", response->error_code.val);
    return false;
  }
  group.setStartStateToCurrentState();
  // IK returns the full robot, including finger mimic joints outside this
  // planning group. The RobotState overload extracts only group variables.
  moveit::core::RobotState solution(group.getRobotModel());
  solution.setToDefaultValues();
  moveit::core::robotStateMsgToRobotState(response->solution, solution);
  if (!solution.satisfiesBounds(solution.getJointModelGroup(group.getName()), 1e-5)) return false;
  return group.setJointValueTarget(solution);
}

bool CompleteTimedPath(double fraction, const moveit_msgs::msg::RobotTrajectory& trajectory) {
  if (!std::isfinite(fraction) || fraction < 0.999 ||
      trajectory.joint_trajectory.points.size() < 2) return false;
  int64_t previous = -1;
  for (const auto& point : trajectory.joint_trajectory.points) {
    const int64_t stamp = int64_t(point.time_from_start.sec) * 1000000000LL +
                          point.time_from_start.nanosec;
    if (stamp <= previous) return false;
    previous = stamp;
  }
  return true;
}

bool OnlyExpectedTargetContacts(const moveit_msgs::srv::GetStateValidity::Response& response,
                                double center_x, double center_y, double min_z, double max_z,
                                double radius) {
  if (response.valid) return true;
  for (const auto& contact : response.contacts) {
    const bool octomap = contact.contact_body_1 == "<octomap>" || contact.contact_body_2 == "<octomap>";
    const std::string& link = contact.contact_body_1 == "<octomap>" ? contact.contact_body_2 : contact.contact_body_1;
    const bool fingertip = link.find("hand_l_distal_link") != std::string::npos ||
                           link.find("hand_r_distal_link") != std::string::npos;
    const double dx = contact.position.x - center_x;
    const double dy = contact.position.y - center_y;
    const bool target = dx * dx + dy * dy <= radius * radius &&
                        contact.position.z >= min_z && contact.position.z <= max_z;
    if (!(octomap && fingertip && target)) return false;
  }
  return !response.contacts.empty();
}

bool SafeBaseState(const moveit::core::RobotState& state) {
  const double x = state.getVariablePosition("odom_x");
  const double y = state.getVariablePosition("odom_y");
  return x >= -0.01 && x <= 0.40 && std::abs(y) <= 0.40;
}

// Build short Cartesian segments using the same collision-aware IK service as
// pregrasp. Humble's Cartesian service crashed in TOTG for this mixed
// prismatic/revolute group; IPTP retains the checked joint-space waypoints.
double CheckedCartesianPath(const rclcpp::Node::SharedPtr& node,
                            moveit::planning_interface::MoveGroupInterface& group,
                            const moveit::core::RobotState& start,
                            const geometry_msgs::msg::Pose& target,
                            moveit_msgs::msg::RobotTrajectory& output,
                            bool allow_target_contacts,
                            double target_x, double target_y, double target_min_z,
                            double target_max_z, double target_radius) {
  if (group.getPoseReferenceFrame() != group.getRobotModel()->getModelFrame()) return 0.0;
  auto client = node->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");
  auto valid_client = node->create_client<moveit_msgs::srv::GetStateValidity>("check_state_validity");
  if (!client->wait_for_service(std::chrono::seconds(5)) ||
      !valid_client->wait_for_service(std::chrono::seconds(5))) return 0.0;
  auto valid = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  valid->group_name = group.getName();
  valid->constraints = group.getPathConstraints();
  moveit::core::robotStateToRobotStateMsg(start, valid->robot_state);
  auto vf = valid_client->async_send_request(valid);
  if (vf.wait_for(std::chrono::seconds(5)) != std::future_status::ready) return 0.0;
  auto start_valid = vf.get();
  // Same allowance the midpoint check below applies. Without it the lift is
  // rejected by the very contacts a closed grasp must have: after closing on
  // the target the fingertips are touching it, so an unconditional check here
  // makes every successful grasp unliftable.
  if (!start_valid->valid && (!allow_target_contacts ||
      !OnlyExpectedTargetContacts(*start_valid, target_x, target_y, target_min_z,
                                  target_max_z, target_radius))) {
    RCLCPP_WARN(node->get_logger(), "Cartesian start state invalid (%zu contacts)", start_valid->contacts.size());
    for (const auto& contact : start_valid->contacts)
      RCLCPP_WARN(node->get_logger(), "  start contact %s / %s", contact.contact_body_1.c_str(), contact.contact_body_2.c_str());
    return 0.0;
  }
  moveit::core::RobotState state(start);
  state.update();
  const auto initial = state.getGlobalLinkTransform("hand_palm_link");
  const Eigen::Vector3d end(target.position.x, target.position.y, target.position.z);
  const Eigen::Quaterniond last(target.orientation.w, target.orientation.x,
                                target.orientation.y, target.orientation.z);
  const Eigen::Quaterniond first(initial.rotation());
  const int steps = std::max(2, static_cast<int>(std::ceil(std::max(
      (end - initial.translation()).norm() / 0.003, first.angularDistance(last) / 0.03))));
  const auto* jmg = group.getRobotModel()->getJointModelGroup(group.getName());
  robot_trajectory::RobotTrajectory path(group.getRobotModel(), group.getName());
  path.addSuffixWayPoint(state, 0.0);
  for (int i = 1; i <= steps; ++i) {
    const double t = double(i) / steps;
    const Eigen::Vector3d p = (1.0 - t) * initial.translation() + t * end;
    const Eigen::Quaterniond q = first.slerp(t, last);
    auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    auto& ik = req->ik_request;
    ik.group_name = group.getName(); ik.ik_link_name = "hand_palm_link";
    moveit::core::robotStateToRobotStateMsg(state, ik.robot_state);
    ik.avoid_collisions = !allow_target_contacts; ik.constraints = group.getPathConstraints(); ik.timeout.sec = 1;
    ik.pose_stamped.header.frame_id = group.getPoseReferenceFrame();
    auto& pose = ik.pose_stamped.pose;
    pose.position.x = p.x(); pose.position.y = p.y(); pose.position.z = p.z();
    pose.orientation.x = q.x(); pose.orientation.y = q.y();
    pose.orientation.z = q.z(); pose.orientation.w = q.w();
    auto future = client->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) return double(i - 1) / steps;
    auto result = future.get();
    if (result->error_code.val != 1) {
      RCLCPP_WARN(node->get_logger(), "Cartesian IK step %d/%d rejected: %d", i, steps, result->error_code.val);
      return double(i - 1) / steps;
    }
    moveit::core::RobotState next(state);
    moveit::core::robotStateMsgToRobotState(result->solution, next);
    if (!next.satisfiesBounds(jmg, 1e-5)) {
      RCLCPP_ERROR(node->get_logger(), "IK returned out-of-bounds joint values");
      return double(i - 1) / steps;
    }
    for (const auto* joint : jmg->getActiveJointModels()) {
      const double limit = joint->getType() == moveit::core::JointModel::PRISMATIC ? 0.02 : 0.15;
      if (joint->distance(state.getJointPositions(joint), next.getJointPositions(joint)) > limit) {
        RCLCPP_WARN(node->get_logger(), "Cartesian step %d/%d jumps joint %s", i, steps, joint->getName().c_str());
        return double(i - 1) / steps;
      }
    }
    // Check the joint-space midpoint that the controller will interpolate.
    moveit::core::RobotState midpoint(state);
    state.interpolate(next, 0.5, midpoint);
    moveit::core::robotStateToRobotStateMsg(midpoint, valid->robot_state);
    vf = valid_client->async_send_request(valid);
    if (vf.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
      return double(i - 1) / steps;
    auto midpoint_valid = vf.get();
    if (!midpoint_valid->valid && (!allow_target_contacts ||
        !OnlyExpectedTargetContacts(*midpoint_valid, target_x, target_y, target_min_z,
                                    target_max_z, target_radius))) {
      RCLCPP_WARN(node->get_logger(), "Cartesian midpoint %d/%d invalid (%zu contacts)", i, steps,
                  midpoint_valid->contacts.size());
      for (const auto& contact : midpoint_valid->contacts)
        RCLCPP_WARN(node->get_logger(), "  %s / %s", contact.contact_body_1.c_str(), contact.contact_body_2.c_str());
      return double(i - 1) / steps;
    }
    state = next;
    path.addSuffixWayPoint(state, 0.0);
  }
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  if (!timing.computeTimeStamps(path, 0.15, 0.15)) return 0.0;
  path.getRobotTrajectoryMsg(output);
  return 1.0;
}

// On the real robot the action only finishes once the fingers stall on the
// object, hence the bounded wait instead of an unbounded .get().
bool CloseGripper(const rclcpp::Node::SharedPtr& node) {
  using GripperApplyEffort = tmc_control_msgs::action::GripperApplyEffort;
  auto client = rclcpp_action::create_client<GripperApplyEffort>(node, "/gripper_controller/grasp");
  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    return false;
  }

  GripperApplyEffort::Goal goal;
  goal.effort = kGraspEffortNm;
  auto handle = client->async_send_goal(goal).get();
  if (!handle) {
    return false;
  }
  auto result = client->async_get_result(handle);
  if (result.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
    return false;
  }
  return result.get().code == rclcpp_action::ResultCode::SUCCEEDED;
}

bool Lift(const rclcpp::Node::SharedPtr& node, moveit::planning_interface::MoveGroupInterface& group,
          const rclcpp::Logger& logger, double target_x, double target_y,
          double target_min_z, double target_max_z, double target_radius) {
  auto pose = group.getCurrentPose().pose;
  pose.position.z += kLiftMeters;

  moveit_msgs::msg::RobotTrajectory trajectory;
  auto lift_state = group.getCurrentState(5.0);
  if (!lift_state) return false;
  double fraction = CheckedCartesianPath(node, group, *lift_state, pose, trajectory,
                                         true, target_x, target_y, target_min_z,
                                         target_max_z + kLiftMeters, target_radius);
  if (CompleteTimedPath(fraction, trajectory)) {
    return static_cast<bool>(group.execute(trajectory));
  }

  // A 5-DOF arm cannot track an exact vertical line, but the telescopic
  // arm_lift_joint moves the hand straight up on its own: raise it in joint
  // space instead.
  RCLCPP_WARN(logger, "only %.0f%% of the Cartesian lift is feasible, raising arm_lift_joint instead",
              fraction * 100.0);
  std::vector<double> joints = group.getCurrentJointValues();
  const std::vector<std::string>& names = group.getJointNames();
  for (size_t j = 0; j < names.size() && j < joints.size(); ++j) {
    if (names[j] == "arm_lift_joint") {
      if (joints[j] + kLiftMeters > 0.69) {
        RCLCPP_ERROR(logger, "insufficient arm_lift_joint travel for the requested lift");
        return false;
      }
      joints[j] += kLiftMeters;
    }
  }
  group.setJointValueTarget(joints);
  return static_cast<bool>(group.move());
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
      "move_to_grasp", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  struct JoinExecutorOnExit {
    rclcpp::executors::SingleThreadedExecutor& executor;
    std::thread& thread;
    ~JoinExecutorOnExit() {
      executor.cancel();
      if (thread.joinable()) thread.join();
    }
  } join_executor{executor, spin_thread};

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
  const double target_x = node->get_parameter("target_center_x").as_double();
  const double target_y = node->get_parameter("target_center_y").as_double();
  const double target_min_z = node->get_parameter("target_min_z").as_double();
  const double target_max_z = node->get_parameter("target_max_z").as_double();
  const double target_radius = node->get_parameter("target_radius").as_double();
  const bool allow_preview_failure = node->get_parameter("allow_preview_failure").as_bool();
  // Park at the validated contact pose with the hand still open, so the real
  // URDF finger placement can be measured against the target before closing
  // perturbs it. Uses the same checks as a real run; it only skips the close.
  const bool stop_before_close = node->get_parameter("stop_before_close").as_bool();
  const double shape_radius = node->get_parameter("target_shape_radius").as_double();
  const bool separate_target = node->get_parameter("exclude_target_after_pregrasp").as_bool() && shape_radius > 0;

  moveit::planning_interface::MoveGroupInterface group(node, group_name);
  group.setEndEffectorLink("hand_palm_link");
  group.setPoseReferenceFrame(frame_id);
  group.setMaxVelocityScalingFactor(0.15);
  group.setMaxAccelerationScalingFactor(0.15);

  // HSR's base is modeled as three ordinary joints (odom_x/odom_y/odom_t),
  // not a virtual planar joint, so setWorkspace() does NOT bound it. Joint
  // path constraints are what actually keep the base parked: without them
  // RRT freely drags the base sideways/into the shelf during the path.
  // Bounds are centered on the current base position, which must therefore
  // be clear of obstacles when this node starts.
  if (group_name != "arm") {
    std::map<std::string, double> current;
    for (const auto& name : group.getJointNames()) {
      current[name] = 0.0;
    }
    const std::vector<double> joints = group.getCurrentJointValues();
    const std::vector<std::string>& names = group.getJointNames();
    for (size_t j = 0; j < names.size() && j < joints.size(); ++j) {
      current[names[j]] = joints[j];
    }
    // Small base adjustments make the 5-DOF arm pose reachable. Bound yaw
    // as well as translation: large intermediate rotations are unnecessary
    // for a nearby grasp and can destabilize the simulated base controller.
    moveit_msgs::msg::Constraints path;
    const std::vector<std::tuple<std::string, double, double>> base_bounds = {
        // These local bounds supplement full robot/environment collision
        // checks; they are not a replacement for navigation or clearance.
        {"odom_x", 0.10, 0.10}, {"odom_y", 0.10, 0.10}, {"odom_t", 0.15, 0.15}};
    for (const auto& [joint, above, below] : base_bounds) {
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name = joint;
      jc.position = current.count(joint) ? current[joint] : 0.0;
      jc.tolerance_above = above;
      jc.tolerance_below = below;
      jc.weight = 1.0;
      path.joint_constraints.push_back(jc);
    }
    group.setPathConstraints(path);
  }

  // The arm alone is 5-DOF, so exact 6-DOF contact poses are generically
  // unreachable without the base's extra joints. When planning arm-only,
  // accept orientation error instead - harmless around a cylindrical can.
  if (group_name == "arm") {
    group.setGoalOrientationTolerance(0.5);
    group.setGoalPositionTolerance(0.01);
    // Goal sampling for a 5-DOF chain needs time; the default 5s window is
    // mostly spent failing IK before a single valid goal state exists.
    group.setPlanningTime(10.0);
  }

  if (!PrepareCollisionScene(node, group.getRobotModel())) {
    RCLCPP_ERROR(logger, "could not prepare collision-checked RGB-D planning scene");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // The previous executor approached with the hand already closed.
  moveit::planning_interface::MoveGroupInterface gripper(node, "gripper");
  gripper.setJointValueTarget("hand_motor_joint", kOpenGripperRadians);
  gripper.setMaxVelocityScalingFactor(0.15);
  const auto hand_state = gripper.getCurrentState();
  if ((!hand_state || std::abs(hand_state->getVariablePosition("hand_motor_joint") -
                              kOpenGripperRadians) > 0.01) && !gripper.move()) {
    RCLCPP_ERROR(logger, "could not open the gripper collision-free");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  auto candidate_pose = [&](size_t i, double offset) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = xs[i] - offset * 2.0 * (qxs[i] * qzs[i] + qys[i] * qws[i]);
    pose.pose.position.y = ys[i] - offset * 2.0 * (qys[i] * qzs[i] - qxs[i] * qws[i]);
    pose.pose.position.z = zs[i] - offset * (1.0 - 2.0 * (qxs[i] * qxs[i] + qys[i] * qys[i]));
    pose.pose.orientation.x = qxs[i]; pose.pose.orientation.y = qys[i];
    pose.pose.orientation.z = qzs[i]; pose.pose.orientation.w = qws[i];
    return pose;
  };
  // GraspGenX score alone can select a large base relocation. Rank reachable
  // pregrasps by base travel first; keep its score as a small tie-breaker.
  std::vector<size_t> order(xs.size());
  std::iota(order.begin(), order.end(), 0);
  std::vector<double> cost(xs.size(), std::numeric_limits<double>::infinity());
  const auto initial_state = group.getCurrentState();
  const auto* jmg = group.getRobotModel()->getJointModelGroup(group_name);
  const auto& variables = jmg->getVariableNames();
  for (size_t i : order) {
    if (!SetCollisionAwareIK(node, group, candidate_pose(i, kPregraspOffsetMeters))) continue;
    std::vector<double> joint_target;
    group.getJointValueTarget(joint_target);
    cost[i] = 0.001 * (1.0 - scores[i]);
    for (size_t j = 0; j < variables.size(); ++j) {
      if (variables[j] == "odom_x" || variables[j] == "odom_y")
        cost[i] += std::abs(joint_target[j] - initial_state->getVariablePosition(variables[j]));
      else if (variables[j] == "odom_t")
        cost[i] += 0.1 * std::abs(std::remainder(joint_target[j] - initial_state->getVariablePosition(variables[j]), 2.0 * M_PI));
    }
  }
  std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return cost[a] < cost[b]; });

  for (size_t i : order) {
    if (!std::isfinite(cost[i])) continue;
    const auto target = candidate_pose(i, 0.0);
    const auto pregrasp = candidate_pose(i, kPregraspOffsetMeters);
    if (!SetCollisionAwareIK(node, group, pregrasp)) {
      RCLCPP_WARN(logger, "grasp %zu: no collision-free pregrasp IK", i + 1);
      continue;
    }

    // Check the complete approach from the proposed IK solution before
    // driving to it. A reachable pregrasp alone is not a feasible grasp.
    auto preview_start = *group.getCurrentState();
    std::vector<double> target_joints;
    group.getJointValueTarget(target_joints);
    preview_start.setJointGroupPositions(group_name, target_joints);
    preview_start.update();
    moveit_msgs::msg::RobotTrajectory preview;
    const double preview_fraction = CheckedCartesianPath(node, group, preview_start, target.pose, preview,
                                                         true, target_x, target_y, target_min_z,
                                                         target_max_z, target_radius);
    if (!CompleteTimedPath(preview_fraction, preview)) {
      RCLCPP_WARN(logger, "grasp %zu: complete approach unavailable (%.0f%%); skipping before motion",
                  i + 1, 100.0 * preview_fraction);
      if (!allow_preview_failure) continue;
      RCLCPP_WARN(logger, "allow_preview_failure enabled: executing the collision-checked pregrasp for visual diagnosis");
    }

    RCLCPP_INFO(logger, "grasp %zu/%zu (score=%.2f): planning '%s' to the pregrasp in %s...",
                i + 1, xs.size(), scores[i], group_name.c_str(), frame_id.c_str());
    auto result = group.move();

    if (result) {
      if (separate_target) {
        RCLCPP_INFO(logger, "pregrasp reached; separating only the target from the RGB-D octomap before contact");
        if (!SeparateTargetAtPregrasp(node, frame_id, target_x, target_y,
                                     target_min_z, target_max_z, shape_radius)) {
          rclcpp::shutdown();
          return EXIT_FAILURE;
        }
      }
      RCLCPP_INFO(logger, "pregrasp reached, approaching the contact pose...");
      bool approached = false;
      moveit_msgs::msg::RobotTrajectory approach;
      double fraction = CheckedCartesianPath(node, group, *group.getCurrentState(), target.pose, approach,
                                             !separate_target, target_x, target_y, target_min_z,
                                             target_max_z, target_radius);
      if (CompleteTimedPath(fraction, approach)) {
        approached = static_cast<bool>(group.execute(approach));
      } else {
        RCLCPP_WARN(logger, "approach incomplete or untimed (%.0f%%); rejecting candidate",
                    fraction * 100.0);
      }
      if (!approached) {
        RCLCPP_WARN(logger, "approach to the contact pose failed");
        if (separate_target) {
          // Do not try another pregrasp with the target already excluded.
          rclcpp::shutdown();
          return EXIT_FAILURE;
        }
        continue;
      }
      if (stop_before_close) {
        RCLCPP_INFO(logger,
                    "reached grasp %zu/%zu; stop_before_close set, parking with the hand open",
                    i + 1, xs.size());
        rclcpp::shutdown();
        return EXIT_SUCCESS;
      }
      RCLCPP_INFO(logger, "reached grasp %zu/%zu, closing the gripper...", i + 1, xs.size());
      if (!CloseGripper(node)) {
        RCLCPP_ERROR(logger, "gripper grasp action failed");
        rclcpp::shutdown();
        return EXIT_FAILURE;
      }

      RCLCPP_INFO(logger, "gripper closed, lifting %.2fm...", kLiftMeters);
      if (!Lift(node, group, logger, target_x, target_y, target_min_z,
                target_max_z, target_radius)) {
        RCLCPP_ERROR(logger, "lift failed; gripper is closed but object capture is unverified");
        rclcpp::shutdown();
        return EXIT_FAILURE;
      }

      RCLCPP_INFO(logger, "grasp %zu/%zu motion sequence completed; verify the object lifted", i + 1, xs.size());
      rclcpp::shutdown();
      return EXIT_SUCCESS;
    }
    RCLCPP_WARN(logger, "grasp %zu/%zu failed (error code %d)", i + 1, xs.size(), result.val);
  }

  RCLCPP_ERROR(logger, "none of the %zu candidate grasps were reachable", xs.size());
  rclcpp::shutdown();
  return EXIT_FAILURE;
}
