#include "agrobot_emra_planner/emra_planner.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"

namespace agrobot_emra_planner
{

namespace
{
template<typename T>
T param(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & key, T def)
{
  nav2_util::declare_parameter_if_not_declared(node, key, rclcpp::ParameterValue(def));
  T v{};
  node->get_parameter(key, v);
  return v;
}
}  // namespace

void EmraPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {throw std::runtime_error("EmraPlanner: unable to lock parent node");}
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  const std::string n = name + ".";
  PlanParams & p = params_;
  p.safe_radius = param<double>(node, n + "safe_radius", 0.17);
  p.escape_dist = param<double>(node, n + "start_escape_dist", 0.5);
  p.goal_tolerance = param<double>(node, n + "goal_tolerance", 0.3);
  p.control_spacing = param<double>(node, n + "control_spacing", 0.75);
  p.max_ctrl = static_cast<int>(param<int>(node, n + "max_control_points", 24));
  p.search_radius = param<double>(node, n + "search_radius", 1.0);
  p.output_spacing = param<double>(node, n + "interpolation_resolution", 0.0);
  allow_unknown_ = param<bool>(node, n + "allow_unknown", false);
  corridor_margin_ = param<double>(node, n + "corridor_margin", 3.0);
  max_roi_cells_ = static_cast<int>(param<int>(node, n + "max_roi_cells", 4000000));
  publish_debug_ = param<bool>(node, n + "publish_debug", true);

  p.seed.timeout_s = param<double>(node, n + "seed_timeout", 1.0);
  p.seed.w_prox = param<double>(node, n + "seed_clearance_weight", 4.0);

  p.cost.w_len = param<double>(node, n + "w_length", 1.0);
  p.cost.w_smooth = param<double>(node, n + "w_smoothness", 0.1);
  p.cost.w_clear = param<double>(node, n + "w_clearance", 0.5);
  p.cost.w_cost = param<double>(node, n + "w_cost", 0.2);
  p.cost.desired_extra = param<double>(node, n + "desired_extra_clearance", 0.25);
  p.cost.opt_margin = param<double>(node, n + "optimizer_margin", 0.03);

  p.emra.pop_size = static_cast<int>(param<int>(node, n + "population_size", 30));
  p.emra.max_iterations = static_cast<int>(param<int>(node, n + "max_iterations", 150));
  p.emra.stall_iterations = static_cast<int>(param<int>(node, n + "stall_iterations", 40));
  p.emra.max_time_s = param<double>(node, n + "max_planning_time", 1.0);
  p.emra.adaptive_iterations = param<bool>(node, n + "adaptive_iterations", true);
  p.emra.num_threads = static_cast<int>(param<int>(node, n + "num_threads", 2));
  p.emra.seed = static_cast<uint64_t>(param<int>(node, n + "random_seed", 0));
  p.emra.greedy_selection = param<bool>(node, n + "greedy_selection", true);
  p.optimize = param<bool>(node, n + "optimize", true);

  seed_pub_ = node->create_publisher<nav_msgs::msg::Path>(name + "/emra_seed", 1);
  stats_pub_ = node->create_publisher<std_msgs::msg::String>(name + "/emra_stats", 1);
  RCLCPP_INFO(
    logger_, "EMRA planner '%s' configured (safe_radius=%.2f m, pop=%d, T_max=%d, budget=%.2f s, threads=%d)",
    name.c_str(), p.safe_radius, p.emra.pop_size, p.emra.max_iterations, p.emra.max_time_s, p.emra.num_threads);
}

void EmraPlanner::cleanup()
{
  seed_pub_.reset();
  stats_pub_.reset();
  RCLCPP_INFO(logger_, "EMRA planner '%s' cleaned up", name_.c_str());
}

void EmraPlanner::activate()
{
  if (seed_pub_) {seed_pub_->on_activate();}
  if (stats_pub_) {stats_pub_->on_activate();}
}

void EmraPlanner::deactivate()
{
  if (seed_pub_) {seed_pub_->on_deactivate();}
  if (stats_pub_) {stats_pub_->on_deactivate();}
}

bool EmraPlanner::snapshot(
  Vec2 start, Vec2 goal, double margin_m, bool whole_map, GridView & g, PlanError & err)
{
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  unsigned int sx, sy, gx, gy;
  if (!costmap_->worldToMap(start.x, start.y, sx, sy)) {err = PlanError::kStartOutside; return false;}
  if (!costmap_->worldToMap(goal.x, goal.y, gx, gy)) {err = PlanError::kGoalOutside; return false;}
  const int W = static_cast<int>(costmap_->getSizeInCellsX()), H = static_cast<int>(costmap_->getSizeInCellsY());
  const double res = costmap_->getResolution();
  int x0 = 0, y0 = 0, x1 = W - 1, y1 = H - 1;
  const int m = static_cast<int>(std::ceil(margin_m / res));
  if (!whole_map) {
    x0 = std::max(0, static_cast<int>(std::min(sx, gx)) - m);
    y0 = std::max(0, static_cast<int>(std::min(sy, gy)) - m);
    x1 = std::min(W - 1, static_cast<int>(std::max(sx, gx)) + m);
    y1 = std::min(H - 1, static_cast<int>(std::max(sy, gy)) + m);
  } else if (static_cast<long>(W) * H > max_roi_cells_) {
    const int mm = static_cast<int>(std::ceil(15.0 / res));   // cap huge maps to +-15 m around the pair
    x0 = std::max(0, static_cast<int>(std::min(sx, gx)) - mm);
    y0 = std::max(0, static_cast<int>(std::min(sy, gy)) - mm);
    x1 = std::min(W - 1, static_cast<int>(std::max(sx, gx)) + mm);
    y1 = std::min(H - 1, static_cast<int>(std::max(sy, gy)) + mm);
  }
  const int w = x1 - x0 + 1, h = y1 - y0 + 1;
  g.init(w, h, res, costmap_->getOriginX() + x0 * res, costmap_->getOriginY() + y0 * res, allow_unknown_);
  const unsigned char * map = costmap_->getCharMap();
  for (int y = 0; y < h; ++y) {
    std::copy(map + static_cast<size_t>(y0 + y) * W + x0, map + static_cast<size_t>(y0 + y) * W + x0 + w,
      g.cost.begin() + static_cast<size_t>(y) * w);
  }
  return true;   // lock released here; optimisation runs on the private copy
}

nav_msgs::msg::Path EmraPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  const Vec2 s{start.pose.position.x, start.pose.position.y};
  const Vec2 e{goal.pose.position.x, goal.pose.position.y};

  PlanResult result;
  bool done = false;
  for (int attempt = 0; attempt < 2 && !done; ++attempt) {
    GridView g;
    PlanError err = PlanError::kNone;
    if (!snapshot(s, e, corridor_margin_, attempt == 1, g, err)) {
      if (err == PlanError::kStartOutside) {
        throw nav2_core::StartOutsideMapBounds("EMRA: start (" + std::to_string(s.x) + "," + std::to_string(s.y) + ") outside costmap");
      }
      throw nav2_core::GoalOutsideMapBounds("EMRA: goal (" + std::to_string(e.x) + "," + std::to_string(e.y) + ") outside costmap");
    }
    g.computeEdt();
    try {
      result = planPath(g, s, e, params_, cancel_checker);
      done = true;
    } catch (const PlanException & ex) {
      // A failed search in the corridor ROI may just mean the route leaves it: retry on the whole map.
      if (ex.code == PlanError::kNoPath && attempt == 0) {continue;}
      switch (ex.code) {
        case PlanError::kStartOccupied: throw nav2_core::StartOccupied(std::string("EMRA: ") + ex.what());
        case PlanError::kGoalOccupied: throw nav2_core::GoalOccupied(std::string("EMRA: ") + ex.what());
        case PlanError::kStartOutside: throw nav2_core::StartOutsideMapBounds(std::string("EMRA: ") + ex.what());
        case PlanError::kGoalOutside: throw nav2_core::GoalOutsideMapBounds(std::string("EMRA: ") + ex.what());
        case PlanError::kCancelled: throw nav2_core::PlannerCancelled("EMRA: planning cancelled");
        default: throw nav2_core::NoValidPathCouldBeFound(std::string("EMRA: ") + ex.what());
      }
    }
  }

  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;
  const size_t n = result.path.size();
  path.poses.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = result.path[i].x;
    ps.pose.position.y = result.path[i].y;
    double yaw;
    if (i + 1 < n) {
      yaw = std::atan2(result.path[i + 1].y - result.path[i].y, result.path[i + 1].x - result.path[i].x);
    } else {
      yaw = tf2::getYaw(goal.pose.orientation);
    }
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    ps.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(ps);
  }
  // Last pose keeps the exact goal orientation.
  if (n > 0) {path.poses.back().pose.orientation = goal.pose.orientation;}

  const PlanStats & st = result.stats;
  RCLCPP_INFO(
    logger_, "EMRA plan: %s | len %.2f m (seed %.2f) | min clearance %.3f m | %d it / %d evals | seed %.3f s emra %.3f s total %.3f s",
    st.source.c_str(), st.final_length, st.seed_length, st.min_clearance, st.iterations, st.evaluations,
    st.seed_time_s, st.emra_time_s, st.total_time_s);

  if (publish_debug_) {
    if (seed_pub_ && seed_pub_->is_activated()) {
      nav_msgs::msg::Path sp;
      sp.header = path.header;
      for (const auto & v : result.seed) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = v.x; ps.pose.position.y = v.y; ps.pose.orientation.w = 1.0;
        sp.poses.push_back(ps);
      }
      seed_pub_->publish(sp);
    }
    if (stats_pub_ && stats_pub_->is_activated()) {
      std::ostringstream os;
      os << "source=" << st.source << " length=" << st.final_length << " seed_length=" << st.seed_length
         << " min_clearance=" << st.min_clearance << " iterations=" << st.iterations
         << " evaluations=" << st.evaluations << " seed_time=" << st.seed_time_s
         << " emra_time=" << st.emra_time_s << " total_time=" << st.total_time_s
         << " emra_feasible=" << st.emra_feasible;
      std_msgs::msg::String m;
      m.data = os.str();
      stats_pub_->publish(m);
    }
  }
  return path;
}

}  // namespace agrobot_emra_planner

PLUGINLIB_EXPORT_CLASS(agrobot_emra_planner::EmraPlanner, nav2_core::GlobalPlanner)
