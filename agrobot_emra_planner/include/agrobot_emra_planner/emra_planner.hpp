#pragma once
#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

#include "agrobot_emra_planner/planning_core.hpp"

namespace agrobot_emra_planner
{

class EmraPlanner : public nav2_core::GlobalPlanner
{
public:
  EmraPlanner() = default;
  ~EmraPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  // Copies the region of interest of the costmap (under the costmap lock) into `g`.
  // Returns false if start or goal lies outside the costmap.
  bool snapshot(Vec2 start, Vec2 goal, double margin_m, bool whole_map, GridView & g, PlanError & err);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("EmraPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_, global_frame_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  PlanParams params_;
  bool allow_unknown_{false};
  double corridor_margin_{3.0};
  int max_roi_cells_{4000000};
  bool publish_debug_{true};

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr seed_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr stats_pub_;
};

}  // namespace agrobot_emra_planner
