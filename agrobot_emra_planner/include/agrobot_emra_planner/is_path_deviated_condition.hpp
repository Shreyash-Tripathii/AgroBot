#pragma once
#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace agrobot_emra_planner
{

// SUCCESS when the robot has drifted more than `max_deviation` metres from `path`
// (or the path is empty); FAILURE otherwise (and when the robot pose is unavailable).
class IsPathDeviatedCondition : public BT::ConditionNode
{
public:
  IsPathDeviatedCondition(const std::string & name, const BT::NodeConfiguration & conf);
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to check against"),
      BT::InputPort<double>("max_deviation", 0.6, "Max allowed distance to the path [m]"),
      BT::InputPort<std::string>("global_frame", "", "Global frame (default: blackboard / map)"),
      BT::InputPort<std::string>("robot_base_frame", "", "Robot base frame (default: blackboard / base_link)")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_{0.1};
  bool initialized_{false};
  std::string global_frame_, base_frame_;
  void init();
};

}  // namespace agrobot_emra_planner
