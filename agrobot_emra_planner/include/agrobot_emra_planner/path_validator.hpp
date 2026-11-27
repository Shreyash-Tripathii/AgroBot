#pragma once
// Single source of truth for "is this path safe". Exact (square-vs-circle) clearance checks.
#include <vector>

#include "agrobot_emra_planner/grid_map_view.hpp"

namespace agrobot_emra_planner
{

class PathValidator
{
public:
  // required: minimum clearance (m) between the robot's circumscribed circle centre and any
  //           blocked cell square.
  // escape_dist: within this Euclidean distance of `start` the requirement is relaxed to
  //           min(required, clearance at start) so the robot can drive away from a wall it is
  //           already close to (cells must still be non-lethal).
  PathValidator(const GridView & g, double required, double escape_dist, Vec2 start);

  bool pointOk(const Vec2 & p) const;
  bool segmentOk(const Vec2 & a, const Vec2 & b) const;

  struct Report
  {
    bool ok{true};
    int first_bad{-1};
  };
  Report checkPolyline(const std::vector<Vec2> & pts) const;

  // Exact minimum clearance along a polyline (capped).
  double minClearance(const std::vector<Vec2> & pts, double cap) const;

  double startClearance() const {return c0_;}
  double step() const {return step_;}

private:
  const GridView & g_;
  double required_;
  double escape_;
  Vec2 start_;
  double c0_;
  double step_;
};

}  // namespace agrobot_emra_planner
