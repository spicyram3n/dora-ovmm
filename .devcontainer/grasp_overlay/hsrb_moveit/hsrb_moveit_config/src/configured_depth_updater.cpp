#include <moveit/depth_image_octomap_updater/depth_image_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <pluginlib/class_list_macros.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <mutex>
#include <cmath>

namespace hsr_rgbd {
// Humble initializes updaters before setParams(). The native depth updater
// configures its GL mesh filter only in initialize(), leaving clipping and
// padding at constructor defaults. Reinitialize before any shapes are added.
class ConfiguredDepthUpdater : public occupancy_map_monitor::DepthImageOctomapUpdater {
 public:
  bool initialize(const rclcpp::Node::SharedPtr& node) override {
    node_ = node;
    if (!DepthImageOctomapUpdater::initialize(node)) return false;
    exclusion_ = node->create_subscription<moveit_msgs::msg::CollisionObject>(
        "/octomap_target_exclusion", 1, [this](moveit_msgs::msg::CollisionObject::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (msg->id != "grasp_target") return;
          if (msg->operation == moveit_msgs::msg::CollisionObject::REMOVE) { active_ = false; return; }
          if (msg->header.frame_id != "odom" || msg->primitives.size() != 1 ||
              msg->primitive_poses.size() != 1 || msg->primitives[0].type != 3 ||
              msg->primitives[0].dimensions.size() != 2) return;
          const auto& p = msg->primitive_poses[0];
          const double h = msg->primitives[0].dimensions[0], r = msg->primitives[0].dimensions[1];
          if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
              !std::isfinite(p.position.z) || !std::isfinite(h) || !std::isfinite(r) ||
              h < .04 || h > .4 || r < .015 || r > .06 ||
              p.orientation.x != 0 || p.orientation.y != 0 || p.orientation.z != 0 || p.orientation.w != 1) return;
          x_ = p.position.x; y_ = p.position.y;
          lo_ = p.position.z - h / 2; hi_ = p.position.z + h / 2; radius_ = r;
          active_ = true;
        });
    cleanup_ = node->create_wall_timer(std::chrono::milliseconds(200), [this]() {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || !tree_ || monitor_->getMapFrame() != "odom") return;
      // Clear voxel centres only within the fitted upright target. Preserve
      // cells at/below its base (including the supporting tabletop). A single
      // voxel of radial tolerance covers depth quantisation at the surface.
      tree_->lockWrite();
      const double step = tree_->getResolution(), r = radius_ + step;
      for (double x = std::floor((x_ - r) / step) * step + step / 2; x < x_ + r; x += step)
        for (double y = std::floor((y_ - r) / step) * step + step / 2; y < y_ + r; y += step) {
          if ((x-x_)*(x-x_) + (y-y_)*(y-y_) > r*r) continue;
          for (double z = std::floor(lo_ / step) * step + step / 2; z <= hi_; z += step) {
            if (z <= lo_) continue;
            auto* existing = tree_->search(x, y, z);
            if (existing && tree_->isNodeOccupied(existing))
              tree_->setNodeValue(tree_->coordToKey(x, y, z), tree_->getClampingThresMinLog(), true);
          }
        }
      tree_->updateInnerOccupancy();
      tree_->unlockWrite();
      tree_->triggerUpdateCallback();
    });
    return true;
  }
  bool setParams(const std::string& ns) override {
    if (!DepthImageOctomapUpdater::setParams(ns)) return false;
    return DepthImageOctomapUpdater::initialize(node_);
  }
 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr exclusion_;
  rclcpp::TimerBase::SharedPtr cleanup_;
  std::mutex mutex_;
  bool active_ = false;
  double x_ = 0, y_ = 0, lo_ = 0, hi_ = 0, radius_ = 0;
};
}
PLUGINLIB_EXPORT_CLASS(hsr_rgbd::ConfiguredDepthUpdater, occupancy_map_monitor::OccupancyMapUpdater)
