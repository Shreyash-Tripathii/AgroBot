// Copyright 2026 Agrobot
//
// Chainable heading-hold controller (port of yaw_heading_controller.py).
//
//   /cmd_vel (Twist)  or  upstream reference  ->  [heading hold on IMU yaw]
//     ->  <output_controller>/linear/velocity , <output_controller>/angular/velocity
//
// Reads <imu_sensor>/orientation.{x,y,z,w} and <imu_sensor>/angular_velocity.z.

#ifndef AGROBOT_CONTROLLERS__YAW_HEADING_CONTROLLER_HPP_
#define AGROBOT_CONTROLLERS__YAW_HEADING_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace agrobot_controllers
{

class YawHeadingController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Cmd
  {
    double vx{0.0};
    double wz{0.0};
    double stamp_s{-1.0e9};  // arrival time; far in the past = "no command yet"
  };

  void write_output(double vx, double wz);

  // parameters
  std::string imu_sensor_;
  std::string output_controller_;
  std::string cmd_vel_topic_;
  bool input_stamped_{false};
  double kp_{2.2}, kd_{0.15}, ki_{0.0};
  double max_angular_vel_{2.0};
  double max_heading_error_{0.785};
  double cmd_timeout_{0.5};
  bool pass_through_without_imu_{false};

  // input
  realtime_tools::RealtimeBuffer<Cmd> cmd_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_sub_;

  // controller state
  bool target_valid_{false};
  double target_yaw_{0.0};
  double integral_error_{0.0};
};

}  // namespace agrobot_controllers

#endif  // AGROBOT_CONTROLLERS__YAW_HEADING_CONTROLLER_HPP_
