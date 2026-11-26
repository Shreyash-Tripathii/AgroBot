// Copyright 2026 Agrobot

#include "agrobot_controllers/fuzzy_gain_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

// See yaw_heading_controller.cpp: the older, still supported ros2_control API is used on purpose.
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace agrobot_controllers
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double finite_or_zero(double v)
{
  return std::isfinite(v) ? v : 0.0;
}

double trimf(double x, double a, double b, double c)
{
  if (x <= a || x >= c) {return 0.0;}
  if (a < x && x <= b) {return b != a ? (x - a) / (b - a) : 1.0;}
  if (b < x && x < c) {return b != c ? (c - x) / (c - b) : 1.0;}
  return 0.0;
}

double trapmf(double x, double a, double b, double c, double d)
{
  if (x <= a || x >= d) {return 0.0;}
  if (a < x && x <= b) {return b != a ? (x - a) / (b - a) : 1.0;}
  if (b < x && x <= c) {return 1.0;}
  if (c < x && x < d) {return c != d ? (d - x) / (d - c) : 1.0;}
  return 0.0;
}
}  // namespace

controller_interface::CallbackReturn FuzzyGainScheduler::on_init()
{
  try {
    auto_declare<std::string>("output_controller", "agrobot_base_controller");
    auto_declare<std::vector<std::string>>(
      "left_wheel_names", {"front_left_wheel_joint", "rear_left_wheel_joint"});
    auto_declare<std::vector<std::string>>(
      "right_wheel_names", {"front_right_wheel_joint", "rear_right_wheel_joint"});
    auto_declare<double>("wheel_radius", 0.0325);     // 65 mm dia
    auto_declare<double>("wheel_separation", 0.22);   // 22 cm track
    auto_declare<double>("schedule_rate_hz", 5.0);
    auto_declare<double>("lpf_alpha", 0.75);
    auto_declare<double>("gain_deadzone", 0.15);      // prevents serial micro-chatter
    auto_declare<double>("turn_threshold", 0.15);     // |w| above this enables the fuzzy boost
    auto fill = [&](double v) {return std::vector<double>(4, v);};
    auto_declare<std::vector<double>>("kp_min", fill(1.5));
    auto_declare<std::vector<double>>("kp_max", fill(18.0));
    auto_declare<std::vector<double>>("ki_min", fill(0.5));
    auto_declare<std::vector<double>>("ki_max", fill(4.0));
    auto_declare<std::vector<double>>("kd_min", fill(0.0));
    auto_declare<std::vector<double>>("kd_max", fill(0.3));

    // Interface configurations are queried right after init, so resolve these now.
    auto node = get_node();
    output_controller_ = node->get_parameter("output_controller").as_string();
    wheels_ = node->get_parameter("left_wheel_names").as_string_array();
    num_left_ = wheels_.size();
    const auto right = node->get_parameter("right_wheel_names").as_string_array();
    wheels_.insert(wheels_.end(), right.begin(), right.end());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FuzzyGainScheduler::command_interface_configuration()
const
{
  std::vector<std::string> names = {
    output_controller_ + "/linear/velocity", output_controller_ + "/angular/velocity"};
  for (const auto & w : wheels_) {
    names.push_back(w + "/kp");
    names.push_back(w + "/ki");
    names.push_back(w + "/kd");
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::InterfaceConfiguration FuzzyGainScheduler::state_interface_configuration()
const
{
  std::vector<std::string> names;
  for (const auto & w : wheels_) {
    names.push_back(w + "/velocity");
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::CallbackReturn FuzzyGainScheduler::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  wheel_radius_ = node->get_parameter("wheel_radius").as_double();
  half_track_ = 0.5 * node->get_parameter("wheel_separation").as_double();
  schedule_rate_hz_ = node->get_parameter("schedule_rate_hz").as_double();
  lpf_alpha_ = node->get_parameter("lpf_alpha").as_double();
  deadzone_ = node->get_parameter("gain_deadzone").as_double();
  turn_threshold_ = node->get_parameter("turn_threshold").as_double();
  kp_min_ = node->get_parameter("kp_min").as_double_array();
  kp_max_ = node->get_parameter("kp_max").as_double_array();
  ki_min_ = node->get_parameter("ki_min").as_double_array();
  ki_max_ = node->get_parameter("ki_max").as_double_array();
  kd_min_ = node->get_parameter("kd_min").as_double_array();
  kd_max_ = node->get_parameter("kd_max").as_double_array();

  const std::size_t n = wheels_.size();
  for (const auto * v : {&kp_min_, &kp_max_, &ki_min_, &ki_max_, &kd_min_, &kd_max_}) {
    if (v->size() != n) {
      RCLCPP_ERROR(node->get_logger(),
        "kp/ki/kd min/max need %zu entries (one per wheel, order [left..., right...]).", n);
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  if (n == 0 || num_left_ == 0 || num_left_ == n || wheel_radius_ <= 0.0 ||
    schedule_rate_hz_ <= 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid wheel configuration or rate.");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FuzzyGainScheduler::on_activate(
  const rclcpp_lifecycle::State &)
{
  const std::size_t n = wheels_.size();
  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), kNaN);
  elapsed_ = 0.0;
  prev_error_.assign(n, 0.0);
  filt_kp_ = pub_kp_ = kp_min_;
  filt_ki_ = pub_ki_ = ki_min_;
  filt_kd_ = pub_kd_ = kd_min_;
  // Gains stay at the ESP32's URDF initial values until the first real change is
  // published (same behaviour as the old node).
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FuzzyGainScheduler::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() >= 2) {
    (void)command_interfaces_[0].set_value(0.0);
    (void)command_interfaces_[1].set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> FuzzyGainScheduler::
on_export_reference_interfaces()
{
  reference_interfaces_.resize(2, kNaN);
  std::vector<hardware_interface::CommandInterface> refs;
  refs.emplace_back(get_node()->get_name(), "linear/velocity", &reference_interfaces_[0]);
  refs.emplace_back(get_node()->get_name(), "angular/velocity", &reference_interfaces_[1]);
  return refs;
}

bool FuzzyGainScheduler::on_set_chained_mode(bool)
{
  return true;
}

controller_interface::return_type FuzzyGainScheduler::update_reference_from_subscribers(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return controller_interface::return_type::OK;  // head-less: references come from the chain
}

double FuzzyGainScheduler::fuzzy_boost(double e, double de)
{
  // Aggressive error memberships (trigger instantly on small lags)
  const double E_Small = trapmf(e, -1.0, 0.0, 0.1, 0.25);
  const double E_Medium = trimf(e, 0.15, 0.5, 1.0);
  const double E_Large = trapmf(e, 0.7, 1.2, 10.0, 10.0);

  // Delta-error memberships
  const double dE_Falling = trapmf(de, -10.0, -10.0, -0.2, -0.05);
  const double dE_Steady = trimf(de, -0.1, 0.0, 0.1);
  const double dE_Rising = trapmf(de, 0.05, 0.2, 10.0, 10.0);

  // Rule weights, biased upward so Kp snaps high when the wheels lag
  const double rules[6][2] = {
    {E_Small, 0.0},
    {std::min(E_Medium, dE_Falling), 0.4},
    {std::min(E_Medium, dE_Steady), 0.8},
    {std::min(E_Medium, dE_Rising), 1.0},
    {std::min(E_Large, dE_Falling), 0.7},
    {std::min(E_Large, std::max(dE_Steady, dE_Rising)), 1.0}};

  double num = 0.0;
  double den = 0.0;
  for (const auto & r : rules) {
    num += r[0] * r[1];
    den += r[0];
  }
  return den > 0.001 ? num / den : 0.0;
}

void FuzzyGainScheduler::tick(double v, double w, double dt)
{
  const std::size_t n = wheels_.size();
  const double left_target = (v - w * half_track_) / wheel_radius_;
  const double right_target = (v + w * half_track_) / wheel_radius_;

  // Turn-intent gate: protects straight-line cruising from fuzzy bursts
  const bool is_turning = std::fabs(w) > turn_threshold_;
  const double dt_safe = std::max(0.001, dt);
  bool requires_publish = false;

  for (std::size_t i = 0; i < n; ++i) {
    const double target = i < num_left_ ? left_target : right_target;
    const double actual = finite_or_zero(state_interfaces_[i].get_value());
    const double e = std::fabs(target - actual);
    const double de = (e - prev_error_[i]) / dt_safe;
    prev_error_[i] = e;

    const double boost = is_turning ? fuzzy_boost(e, de) : 0.0;
    const double raw_kp = kp_min_[i] + boost * (kp_max_[i] - kp_min_[i]);
    const double raw_ki = ki_min_[i] + boost * (ki_max_[i] - ki_min_[i]);
    const double raw_kd = kd_min_[i] + boost * (kd_max_[i] - kd_min_[i]);

    const double a = lpf_alpha_;
    filt_kp_[i] = a * raw_kp + (1.0 - a) * filt_kp_[i];
    filt_ki_[i] = a * raw_ki + (1.0 - a) * filt_ki_[i];
    filt_kd_[i] = a * raw_kd + (1.0 - a) * filt_kd_[i];

    // Deadzone quantisation
    if (std::fabs(filt_kp_[i] - pub_kp_[i]) > deadzone_ ||
      std::fabs(filt_ki_[i] - pub_ki_[i]) > deadzone_ ||
      std::fabs(filt_kd_[i] - pub_kd_[i]) > deadzone_)
    {
      pub_kp_[i] = std::round(filt_kp_[i] * 100.0) / 100.0;
      pub_ki_[i] = std::round(filt_ki_[i] * 100.0) / 100.0;
      pub_kd_[i] = std::round(filt_kd_[i] * 100.0) / 100.0;
      requires_publish = true;
    }
  }

  if (requires_publish) {
    for (std::size_t i = 0; i < n; ++i) {
      (void)command_interfaces_[2 + 3 * i + 0].set_value(pub_kp_[i]);
      (void)command_interfaces_[2 + 3 * i + 1].set_value(pub_ki_[i]);
      (void)command_interfaces_[2 + 3 * i + 2].set_value(pub_kd_[i]);
    }
  }
}

controller_interface::return_type FuzzyGainScheduler::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double v = finite_or_zero(reference_interfaces_[0]);
  const double w = finite_or_zero(reference_interfaces_[1]);

  // Pass the velocity command straight through to the drive controller (no topic hop).
  (void)command_interfaces_[0].set_value(v);
  (void)command_interfaces_[1].set_value(w);

  const double dt = period.seconds();
  elapsed_ += dt;
  const double interval = 1.0 / schedule_rate_hz_;
  if (elapsed_ + 0.5 * dt >= interval) {  // decimate the 50 Hz loop to ~5 Hz
    tick(v, w, elapsed_);
    elapsed_ = 0.0;
  }
  return controller_interface::return_type::OK;
}

}  // namespace agrobot_controllers

PLUGINLIB_EXPORT_CLASS(
  agrobot_controllers::FuzzyGainScheduler, controller_interface::ChainableControllerInterface)
