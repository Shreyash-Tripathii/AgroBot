// Copyright 2026 Agrobot
//
// Chainable fuzzy PID gain scheduler (port of gain_scheduler_node.py).
//
//   upstream reference (linear/angular velocity)
//     -> passed through to <output_controller>/linear|angular/velocity
//     -> every 1/schedule_rate_hz: per-wheel fuzzy (error, error-rate) boost
//        -> LPF -> deadzone -> <wheel>/{kp,ki,kd} command interfaces (the ESP32 PID)
//
// Reads <wheel>/velocity. Wheels are ordered [left..., right...].

#ifndef AGROBOT_CONTROLLERS__FUZZY_GAIN_SCHEDULER_HPP_
#define AGROBOT_CONTROLLERS__FUZZY_GAIN_SCHEDULER_HPP_

#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"

namespace agrobot_controllers
{

class FuzzyGainScheduler : public controller_interface::ChainableControllerInterface
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
  void tick(double v, double w, double dt);
  static double fuzzy_boost(double e, double de);

  // parameters
  std::string output_controller_;
  std::vector<std::string> wheels_;  // [left..., right...]
  std::size_t num_left_{0};
  double wheel_radius_{0.0325};
  double half_track_{0.11};
  double schedule_rate_hz_{5.0};
  double lpf_alpha_{0.75};
  double deadzone_{0.15};
  double turn_threshold_{0.15};
  std::vector<double> kp_min_, kp_max_, ki_min_, ki_max_, kd_min_, kd_max_;

  // state (per wheel)
  double elapsed_{0.0};
  std::vector<double> prev_error_;
  std::vector<double> filt_kp_, filt_ki_, filt_kd_;
  std::vector<double> pub_kp_, pub_ki_, pub_kd_;
};

}  // namespace agrobot_controllers

#endif  // AGROBOT_CONTROLLERS__FUZZY_GAIN_SCHEDULER_HPP_
