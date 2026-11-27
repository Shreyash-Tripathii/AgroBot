#pragma once
// ROS-free snapshot of a (region of a) costmap with a Euclidean distance transform.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace agrobot_emra_planner
{

constexpr uint8_t kCostInscribed = 253;
constexpr uint8_t kCostLethal = 254;
constexpr uint8_t kCostUnknown = 255;

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

inline double dist(const Vec2 & a, const Vec2 & b) {return std::hypot(a.x - b.x, a.y - b.y);}

class GridView
{
public:
  int w{0}, h{0};
  double res{0.05};
  double ox{0.0}, oy{0.0};          // world coords of the lower-left corner of cell (0,0)
  bool allow_unknown{false};
  std::vector<uint8_t> cost;        // w*h, row-major (y*w+x), Nav2 cost values
  std::vector<float> edt;           // metres: cell-centre distance to nearest blocked cell / border

  void init(int width, int height, double resolution, double origin_x, double origin_y, bool allow_unk);

  int idx(int x, int y) const {return y * w + x;}
  bool inside(int x, int y) const {return x >= 0 && y >= 0 && x < w && y < h;}
  bool worldToCell(double wx, double wy, int & cx, int & cy) const;
  Vec2 cellCenter(int cx, int cy) const {return {ox + (cx + 0.5) * res, oy + (cy + 0.5) * res};}

  // A cell the robot can never touch: lethal, or unknown when unknown space is not allowed.
  bool blockedCell(int x, int y) const;

  // Must be called after cost has been filled.
  void computeEdt();

  // Fast, approximate clearance (bilinear EDT minus half a cell). <0 outside / inside obstacles.
  // Used by the optimiser's objective; NEVER used as the final safety authority.
  double clearanceEst(double px, double py) const;

  // Exact distance from a point to the nearest blocked cell *square* (or the grid border),
  // capped at `cap`. Cells outside the view are treated as blocked.
  double exactClearance(double px, double py, double cap) const;
};

}  // namespace agrobot_emra_planner
