// Copyright 2026 Agrobot

#include "agrobot_controllers/yaw_heading_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

// ros2_control 4.4x deprecates the get_value()/set_value(double) and
// on_export_reference_interfaces() API in favour of optional/shared-ptr variants. The old
// calls work on every Jazzy 4.x release, so they are used deliberately.
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace agrobot_controllers
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double normalize_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double yaw_from_quaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double finite_or_zero(double v)
{
  return std::isfinite(v) ? v : 0.0;
}
}  // namespace

controller_interface::CallbackReturn YawHeadingController::on_init()
{
  try {
    auto_declare<std::string>("imu_sensor", "imu_game");
    auto_declare<std::string>("output_controller", "fuzzy_gain_scheduler");
    auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
    auto_declare<bool>("input_stamped", false);
    auto_declare<double>("kp", 2.2);
    auto_declare<double>("kd", 0.15);
    auto_declare<double>("ki", 0.0);
    auto_declare<double>("max_angular_vel", 2.0);
    auto_declare<double>("max_heading_error", 0.785);  // ~45 deg anti-windup clamp
    auto_declare<double>("cmd_timeout", 0.5);
    auto_declare<bool>("pass_through_without_imu", false);
    // The interface configurations are queried right after init, so resolve these now.
    imu_sensor_ = get_node()->get_parameter("imu_sensor").as_string();
    output_controller_ = get_node()->get_parameter("output_controller").as_string();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration YawHeadingController::command_interface_configuration()
const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {output_controller_ + "/linear/velocity", output_controller_ + "/angular/velocity"}};
}

controller_interface::InterfaceConfiguration YawHeadingController::state_interface_configuration()
const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {imu_sensor_ + "/orientation.x", imu_sensor_ + "/orientation.y",
      imu_sensor_ + "/orientation.z", imu_sensor_ + "/orientation.w",
      imu_sensor_ + "/angular_velocity.z"}};
}

controller_interface::CallbackReturn YawHeadingController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  imu_sensor_ = node->get_parameter("imu_sensor").as_string();
  output_controller_ = node->get_parameter("output_controller").as_string();
  cmd_vel_topic_ = node->get_parameter("cmd_vel_topic").as_string();
  input_stamped_ = node->get_parameter("input_stamped").as_bool();
  kp_ = node->get_parameter("kp").as_double();
  kd_ = node->get_parameter("kd").as_double();
  ki_ = node->get_parameter("ki").as_double();
  max_angular_vel_ = node->get_parameter("max_angular_vel").as_double();
  max_heading_error_ = node->get_parameter("max_heading_error").as_double();
  cmd_timeout_ = node->get_parameter("cmd_timeout").as_double();
  pass_through_without_imu_ = node->get_parameter("pass_through_without_imu").as_bool();

  cmd_buffer_.writeFromNonRT(Cmd{});

  twist_sub_.reset();
  twist_stamped_sub_.reset();
  if (input_stamped_) {
    twist_stamped_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        cmd_buffer_.writeFromNonRT(
          Cmd{msg->twist.linear.x, msg->twist.angular.z, get_node()->now().seconds()});
      });
  } else {
    twist_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_buffer_.writeFromNonRT(
          Cmd{msg->linear.x, msg->angular.z, get_node()->now().seconds()});
      });
  }

  RCLCPP_INFO(node->get_logger(), "Heading hold on '%s' -> '%s' (cmd on %s, %s)",
    imu_sensor_.c_str(), output_controller_.c_str(), cmd_vel_topic_.c_str(),
    input_stamped_ ? "TwistStamped" : "Twist");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn YawHeadingController::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), kNaN);
  target_valid_ = false;
  integral_error_ = 0.0;
  cmd_buffer_.writeFromNonRT(Cmd{});  // ignore anything received before activation
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn YawHeadingController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  write_output(0.0, 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> YawHeadingController::
on_export_reference_interfaces()
{
  reference_interfaces_.resize(2, kNaN);
  std::vector<hardware_interface::CommandInterface> refs;
  refs.emplace_back(get_node()->get_name(), "linear/velocity", &reference_interfaces_[0]);
  refs.emplace_back(get_node()->get_name(), "angular/velocity", &reference_interfaces_[1]);
  return refs;
}

bool YawHeadingController::on_set_chained_mode(bool)
{
  return true;
}

controller_interface::return_type YawHeadingController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  const Cmd c = *cmd_buffer_.readFromRT();
  const bool timed_out = (time.seconds() - c.stamp_s) > cmd_timeout_;
  reference_interfaces_[0] = timed_out ? 0.0 : c.vx;
  reference_interfaces_[1] = timed_out ? 0.0 : c.wz;
  return controller_interface::return_type::OK;
}

void YawHeadingController::write_output(double vx, double wz)
{
  if (command_interfaces_.size() < 2) {
    return;
  }
  (void)command_interfaces_[0].set_value(vx);
  (void)command_interfaces_[1].set_value(wz);
}

controller_interface::return_type YawHeadingController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double vx = finite_or_zero(reference_interfaces_[0]);
  const double wz = finite_or_zero(reference_interfaces_[1]);
  const double dt = std::clamp(period.seconds(), 0.0, 0.5);

  const double qx = state_interfaces_[0].get_value();
  const double qy = state_interfaces_[1].get_value();
  const double qz = state_interfaces_[2].get_value();
  const double qw = state_interfaces_[3].get_value();
  const double gyro_z = state_interfaces_[4].get_value();

  if (!std::isfinite(qx + qy + qz + qw + gyro_z)) {
    // No (fresh) IMU: never hold a stale heading. Either stop or drive open loop.
    target_valid_ = false;
    integral_error_ = 0.0;
    if (pass_through_without_imu_) {
      write_output(vx, wz);
    } else {
      write_output(0.0, 0.0);
    }
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "No valid IMU data from '%s': %s.", imu_sensor_.c_str(),
      pass_through_without_imu_ ? "driving without heading hold" : "holding the robot stopped");
    return controller_interface::return_type::OK;
  }

  const double yaw = yaw_from_quaternion(qx, qy, qz, qw);
  if (!target_valid_) {
    target_yaw_ = yaw;  // lock the initial heading
    target_valid_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Initial heading locked at %.2f deg",
      yaw * 180.0 / M_PI);
  }

  const bool stopped = std::fabs(vx) < 1e-4 && std::fabs(wz) < 1e-4;
  if (stopped) {
    // Follow external pushes while idle so the robot does not snap back afterwards.
    target_yaw_ = yaw;
    integral_error_ = 0.0;
    write_output(0.0, 0.0);
    return controller_interface::return_type::OK;
  }

  // 1. integrate the desired yaw rate into the target heading
  target_yaw_ = normalize_angle(target_yaw_ + wz * dt);

  // 2. shortest-path heading error, clamped so a blocked robot does not wind up
  double yaw_error = normalize_angle(target_yaw_ - yaw);
  if (std::fabs(yaw_error) > max_heading_error_) {
    yaw_error = std::copysign(max_heading_error_, yaw_error);
    target_yaw_ = normalize_angle(yaw + yaw_error);
  }

  // 3. PID + feed-forward
  integral_error_ = std::clamp(integral_error_ + yaw_error * dt, -0.5, 0.5);
  const double rate_error = wz - gyro_z;
  double out = wz + kp_ * yaw_error + kd_ * rate_error + ki_ * integral_error_;

  // 4. clamp and write
  out = std::clamp(out, -max_angular_vel_, max_angular_vel_);
  write_output(vx, out);
  return controller_interface::return_type::OK;
}

}  // namespace agrobot_controllers

PLUGINLIB_EXPORT_CLASS(
  agrobot_controllers::YawHeadingController, controller_interface::ChainableControllerInterface)
