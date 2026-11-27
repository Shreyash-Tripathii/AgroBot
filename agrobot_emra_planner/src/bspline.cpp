#include "agrobot_emra_planner/bspline.hpp"

#include <algorithm>

namespace agrobot_emra_planner
{
namespace
{

double deBoor(const std::vector<double> & t, int p, const std::vector<double> & c, double u)
{
  const int n = static_cast<int>(c.size());
  int k = p;
  while (k < n - 1 && u >= t[k + 1]) {++k;}
  std::vector<double> d(p + 1);
  for (int j = 0; j <= p; ++j) {d[j] = c[j + k - p];}
  for (int r = 1; r <= p; ++r) {
    for (int j = p; j >= r; --j) {
      const double den = t[j + 1 + k - r] - t[j + k - p];
      const double alpha = den > 0.0 ? (u - t[j + k - p]) / den : 0.0;
      d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
    }
  }
  return d[p];
}

// Control points / knots of the derivative spline.
void derivative(
  const std::vector<double> & t, int p, const std::vector<double> & c,
  std::vector<double> & t2, std::vector<double> & c2)
{
  const int n = static_cast<int>(c.size());
  c2.resize(n - 1);
  for (int i = 0; i < n - 1; ++i) {
    const double den = t[i + p + 1] - t[i + 1];
    c2[i] = den > 0.0 ? p * (c[i + 1] - c[i]) / den : 0.0;
  }
  t2.assign(t.begin() + 1, t.end() - 1);
}

}  // namespace

BSplineBasis::BSplineBasis(int n_ctrl, int n_samples)
: n(n_ctrl), m(n_samples)
{
  const int p = 3;
  umax = static_cast<double>(n - p);
  du = umax / (m - 1);
  std::vector<double> t(n + p + 1);
  for (int i = 0; i < n + p + 1; ++i) {
    t[i] = (i <= p) ? 0.0 : (i >= n ? umax : static_cast<double>(i - p));
  }
  j0.resize(m); w0.resize(m); w1.resize(m); w2.resize(m);
  for (int s = 0; s < m; ++s) {
    const double u = (s == m - 1) ? umax : s * du;
    int k = p;
    while (k < n - 1 && u >= t[k + 1]) {++k;}
    j0[s] = k - p;
    for (int q = 0; q < 4; ++q) {
      std::vector<double> e(n, 0.0);
      e[j0[s] + q] = 1.0;
      w0[s][q] = deBoor(t, p, e, u);
      std::vector<double> t1, e1, t2, e2;
      derivative(t, p, e, t1, e1);
      w1[s][q] = deBoor(t1, p - 1, e1, u);
      derivative(t1, p - 1, e1, t2, e2);
      w2[s][q] = deBoor(t2, p - 2, e2, u);
    }
  }
}

}  // namespace agrobot_emra_planner
