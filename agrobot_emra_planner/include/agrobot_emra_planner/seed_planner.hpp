#pragma once
// Cost- and clearance-aware A* + line-of-sight string pulling. Produces the *guaranteed valid*
// seed path that EMRA then improves. The result is re-verified by PathValidator before return.
#include <functional>
#include <vector>

#include "agrobot_emra_planner/grid_map_view.hpp"
#include "agrobot_emra_planner/path_validator.hpp"

namespace agrobot_emra_planner
{

struct SeedParams
{
  double safe_radius{0.17};
  double escape_dist{0.5};
  double pref_extra{0.30};      // A* repels from walls up to safe_radius + pref_extra
  double w_prox{4.0};           // strength of that repulsion
  double w_cost{0.5};           // weight of costmap cost
  double timeout_s{1.0};
  std::vector<double> margins{0.20, 0.10, 0.04, 0.0};  // string-pulling extra clearance, tried in order
};

struct SeedResult
{
  bool ok{false};
  bool cancelled{false};
  std::vector<Vec2> poly;       // string-pulled, validated
  std::vector<Vec2> chain;      // raw A* cell chain
  double margin_used{0.0};
  int expansions{0};
};

SeedResult planSeed(
  const GridView & g, Vec2 start, Vec2 goal, const SeedParams & p,
  const std::function<bool()> & cancel);

}  // namespace agrobot_emra_planner
