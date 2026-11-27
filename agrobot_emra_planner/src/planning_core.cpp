#include "agrobot_emra_planner/planning_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "agrobot_emra_planner/path_validator.hpp"

namespace agrobot_emra_planner
{

namespace
{
using clock = std::chrono::steady_clock;
double since(const clock::time_point & t) {return std::chrono::duration<double>(clock::now() - t).count();}
}  // namespace

PlanResult planPath(
  const GridView & g, Vec2 start, Vec2 goal, const PlanParams & pin,
  const std::function<bool()> & cancel)
{
  const auto t_all = clock::now();
  PlanParams p = pin;
  p.seed.safe_radius = p.cost.safe_radius = p.safe_radius;
  p.seed.escape_dist = p.cost.escape_dist = p.escape_dist;
  const double spacing = p.output_spacing > 0.0 ? p.output_spacing : g.res;

  int cx, cy;
  if (!g.worldToCell(start.x, start.y, cx, cy)) {
    throw PlanException(PlanError::kStartOutside, "start outside planning region");
  }
  if (g.blockedCell(cx, cy)) {throw PlanException(PlanError::kStartOccupied, "start cell is lethal/unknown");}
  if (!g.worldToCell(goal.x, goal.y, cx, cy)) {
    throw PlanException(PlanError::kGoalOutside, "goal outside planning region");
  }

  const PathValidator validator(g, p.safe_radius, p.escape_dist, start);

  // ---- goal adjustment: nearest safe point within goal_tolerance ----
  if (!validator.pointOk(goal)) {
    bool found = false;
    Vec2 best = goal;
    double bd = p.goal_tolerance + 1e-9;
    const int r = static_cast<int>(std::ceil(p.goal_tolerance / g.res));
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        const int x = cx + dx, y = cy + dy;
        if (!g.inside(x, y)) {continue;}
        const Vec2 c = g.cellCenter(x, y);
        const double d = dist(c, goal);
        if (d < bd && validator.pointOk(c)) {bd = d; best = c; found = true;}
      }
    }
    if (!found) {throw PlanException(PlanError::kGoalOccupied, "goal (and tolerance region) not safe");}
    goal = best;
  }

  PlanResult out;
  out.stats.goal_used = goal;

  if (dist(start, goal) < 0.5 * g.res) {
    out.path = {start, goal};
    out.stats.source = "trivial";
    out.stats.total_time_s = since(t_all);
    out.stats.min_clearance = validator.minClearance(out.path, 1.0);
    return out;
  }

  // ---- guaranteed-valid seed ----
  const auto t_seed = clock::now();
  SeedResult seed = planSeed(g, start, goal, p.seed, cancel);
  out.stats.seed_time_s = since(t_seed);
  if (seed.cancelled) {throw PlanException(PlanError::kCancelled, "cancelled during seed search");}
  if (!seed.ok) {throw PlanException(PlanError::kNoPath, "no collision-free path found");}
  out.seed = seed.poly;
  out.stats.seed_length = polylineLength(seed.poly);

  const std::vector<Vec2> seed_dense = densify(seed.poly, spacing);
  auto finish = [&](std::vector<Vec2> path, const std::string & src) {
      out.path = std::move(path);
      out.stats.source = src;
      out.stats.final_length = polylineLength(out.path);
      out.stats.min_clearance = validator.minClearance(out.path, 1.0);
      out.stats.total_time_s = since(t_all);
      return out;
    };

  if (!p.optimize || out.stats.seed_length < p.min_opt_length) {return finish(seed_dense, "seed_polyline");}

  // ---- EMRA ----
  const double Ls = out.stats.seed_length;
  const int N = std::clamp(static_cast<int>(std::lround(Ls / p.control_spacing)), p.min_ctrl, p.max_ctrl);
  const int M = std::clamp(static_cast<int>(std::ceil(Ls * 1.3 / 0.08)) + 1, 40, 400);
  out.stats.n_ctrl = N;
  PathProblem problem(g, seed.poly, start, goal, p.cost, N, M);

  const int dim = problem.dim();
  std::vector<double> lb(dim, -p.search_radius), ub(dim, p.search_radius);
  const std::vector<double> zero(dim, 0.0);
  EmraResult er = emraOptimize(
    [&](const double * x) {return problem.evaluate(x);}, lb, ub, p.emra, {zero}, cancel);
  if (er.cancelled) {throw PlanException(PlanError::kCancelled, "cancelled during EMRA");}
  out.stats.emra_time_s = er.elapsed_s;
  out.stats.iterations = er.iterations;
  out.stats.evaluations = er.evaluations;
  out.stats.emra_feasible = er.eval.violation <= 1e-9;

  // ---- candidate selection: only exactly-validated paths may be returned ----
  struct Cand {std::vector<Vec2> path; double score; std::string src;};
  std::vector<Cand> valid;
  auto consider = [&](std::vector<Vec2> path, const std::string & src) {
      if (validator.checkPolyline(path).ok) {
        const double sc = scoreDense(g, path, start, p.cost);
        valid.push_back({std::move(path), sc, src});
      }
    };
  const double alphas[4] = {1.0, 0.7, 0.4, 0.0};
  const char * names[4] = {"emra", "emra_scaled", "emra_scaled", "spline_seed"};
  for (int k = 0; k < 4; ++k) {
    std::vector<double> x(er.best);
    for (double & v : x) {v *= alphas[k];}
    consider(problem.decode(x.data(), spacing), names[k]);
  }
  consider(seed_dense, "seed_polyline");
  if (valid.empty()) {throw PlanException(PlanError::kNoPath, "no candidate passed validation");}
  auto bestIt = std::min_element(valid.begin(), valid.end(), [](const Cand & a, const Cand & b) {return a.score < b.score;});
  return finish(bestIt->path, bestIt->src);
}

}  // namespace agrobot_emra_planner
