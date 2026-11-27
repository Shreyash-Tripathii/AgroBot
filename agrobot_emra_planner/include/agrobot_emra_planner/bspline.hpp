#pragma once
// Clamped uniform cubic B-spline with precomputed sparse basis weights at uniform parameters.
#include <array>
#include <vector>

namespace agrobot_emra_planner
{

struct BSplineBasis
{
  int n{0};                         // number of control points (>= 4)
  int m{0};                         // number of samples
  double umax{0.0};
  double du{0.0};
  std::vector<int> j0;              // first control index influencing sample
  std::vector<std::array<double, 4>> w0, w1, w2;   // position / 1st / 2nd derivative weights

  BSplineBasis(int n_ctrl, int n_samples);
};

}  // namespace agrobot_emra_planner
