#pragma once
// End-to-end ROS-free planning pipeline: validate -> seed -> EMRA -> select validated best.
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include "agrobot_emra_planner/emra_optimizer.hpp"
#include "agrobot_emra_planner/grid_map_view.hpp"
#include "agrobot_emra_planner/path_problem.hpp"
#include "agrobot_emra_planner/seed_planner.hpp"

namespace agrobot_emra_planner
{

struct PlanParams
{
  double safe_radius{0.17};
  double escape_dist{0.5};
  double goal_tolerance{0.3};
  double control_spacing{0.75};
  int min_ctrl{3};
  int max_ctrl{24};
  double search_radius{1.0};
  double output_spacing{0.0};     // 0 -> grid resolution
  double min_opt_length{0.4};     // shorter seeds are returned as-is
  SeedParams seed;
  CostParams cost;
  EmraParams emra;
  bool optimize{true};            // false => seed only (used by the benchmark)
};

enum class PlanError {kNone, kStartOutside, kGoalOutside, kStartOccupied, kGoalOccupied, kNoPath, kCancelled};

class PlanException : public std::runtime_error
{
public:
  PlanException(PlanError e, const std::string & m) : std::runtime_error(m), code(e) {}
  PlanError code;
};

struct PlanStats
{
  std::string source;             // emra | emra_scaled | spline_seed | seed_polyline | trivial
  double seed_time_s{0}, emra_time_s{0}, total_time_s{0};
  int iterations{0}, evaluations{0};
  double seed_length{0}, final_length{0};
  double min_clearance{0};        // exact
  int n_ctrl{0};
  bool emra_feasible{false};
  Vec2 goal_used;
};

struct PlanResult
{
  std::vector<Vec2> path;         // dense, validated
  std::vector<Vec2> seed;         // string-pulled seed polyline
  PlanStats stats;
};

PlanResult planPath(
  const GridView & g, Vec2 start, Vec2 goal, const PlanParams & p,
  const std::function<bool()> & cancel);

}  // namespace agrobot_emra_planner
