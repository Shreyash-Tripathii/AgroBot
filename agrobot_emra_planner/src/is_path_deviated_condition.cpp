#include "agrobot_emra_planner/is_path_deviated_condition.hpp"

#include <algorithm>
#include <cmath>

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_util/robot_utils.hpp"

namespace agrobot_emra_planner
{

IsPathDeviatedCondition::IsPathDeviatedCondition(const std::string & name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf) {}

void IsPathDeviatedCondition::init()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  try {
    transform_tolerance_ = node_->get_parameter("transform_tolerance").as_double();
  } catch (...) {transform_tolerance_ = 0.1;}
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", base_frame_);
  // The BT client node carries copies of bt_navigator's parameters.
  if (global_frame_.empty()) {
    try {global_frame_ = node_->get_parameter("global_frame").as_string();} catch (...) {global_frame_ = "map";}
  }
  if (base_frame_.empty()) {
    try {base_frame_ = node_->get_parameter("robot_base_frame").as_string();} catch (...) {base_frame_ = "base_link";}
  }
  initialized_ = true;
}

BT::NodeStatus IsPathDeviatedCondition::tick()
{
  if (!initialized_) {init();}
  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.poses.empty()) {return BT::NodeStatus::SUCCESS;}
  double max_dev = 0.6;
  getInput("max_deviation", max_dev);

  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(pose, *tf_, global_frame_, base_frame_, transform_tolerance_)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "IsPathDeviated: no robot pose");
    return BT::NodeStatus::FAILURE;
  }
  const double px = pose.pose.position.x, py = pose.pose.position.y;
  double best = 1e18;
  const auto & P = path.poses;
  if (P.size() == 1) {
    best = std::hypot(px - P[0].pose.position.x, py - P[0].pose.position.y);
  }
  for (size_t i = 1; i < P.size(); ++i) {
    const double ax = P[i - 1].pose.position.x, ay = P[i - 1].pose.position.y;
    const double bx = P[i].pose.position.x, by = P[i].pose.position.y;
    const double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
    const double t = l2 > 1e-12 ? std::clamp(((px - ax) * dx + (py - ay) * dy) / l2, 0.0, 1.0) : 0.0;
    best = std::min(best, std::hypot(px - (ax + t * dx), py - (ay + t * dy)));
  }
  return best > max_dev ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace agrobot_emra_planner

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<agrobot_emra_planner::IsPathDeviatedCondition>("IsPathDeviated");
}
