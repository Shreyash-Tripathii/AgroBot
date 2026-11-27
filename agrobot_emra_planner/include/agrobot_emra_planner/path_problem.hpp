#pragma once
// Encodes a path as displacements of B-spline control points from a validated seed path and
// scores it (length + smoothness + clearance + cost). Evaluate() is thread-safe.
#include <memory>
#include <vector>

#include "agrobot_emra_planner/bspline.hpp"
#include "agrobot_emra_planner/emra_optimizer.hpp"
#include "agrobot_emra_planner/grid_map_view.hpp"

namespace agrobot_emra_planner
{

struct CostParams
{
  double w_len{1.0};
  double w_smooth{0.1};        // weight of integral of curvature^2 ds
  double w_clear{0.5};         // weight of squared clearance deficit below `desired_extra`
  double w_cost{0.2};          // weight of costmap cost integral
  double safe_radius{0.17};
  double opt_margin{0.03};     // extra clearance the optimiser demands (exact validator decides)
  double desired_extra{0.25};  // soft preferred clearance above safe_radius
  double escape_dist{0.5};
};

// Resample a polyline at (approximately) constant arc-length spacing; keeps first/last points.
std::vector<Vec2> densify(const std::vector<Vec2> & poly, double spacing);
double polylineLength(const std::vector<Vec2> & poly);

// Score an already-dense path with the same terms as the optimiser objective.
double scoreDense(const GridView & g, const std::vector<Vec2> & dense, Vec2 start, const CostParams & cp);

class PathProblem
{
public:
  PathProblem(
    const GridView & g, const std::vector<Vec2> & seed_poly, Vec2 start, Vec2 goal,
    const CostParams & cp, int n_interior, int n_samples);

  int dim() const {return 2 * n_int_;}
  Evaluation evaluate(const double * x) const;
  // Dense, arc-length-spaced path for decision vector x (x may be null => zero displacement).
  std::vector<Vec2> decode(const double * x, double spacing) const;

private:
  void controlPoints(const double * x, double * cx, double * cy) const;

  const GridView & g_;
  CostParams cp_;
  Vec2 start_, goal_;
  int n_int_;
  std::vector<Vec2> base_;     // seed-derived interior control points
  BSplineBasis basis_;
  double c0_est_;              // estimated clearance at start (for the escape zone)
  double seed_len_;
};

}  // namespace agrobot_emra_planner
