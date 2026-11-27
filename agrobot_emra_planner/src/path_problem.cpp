#include "agrobot_emra_planner/path_problem.hpp"

#include <algorithm>
#include <cmath>

namespace agrobot_emra_planner
{

double polylineLength(const std::vector<Vec2> & poly)
{
  double L = 0.0;
  for (size_t i = 1; i < poly.size(); ++i) {L += dist(poly[i - 1], poly[i]);}
  return L;
}

std::vector<Vec2> densify(const std::vector<Vec2> & poly, double spacing)
{
  if (poly.size() < 2) {return poly;}
  const double L = polylineLength(poly);
  const int n = std::max(1, static_cast<int>(std::ceil(L / spacing)));
  const double ds = L / n;
  std::vector<Vec2> out;
  out.reserve(n + 1);
  out.push_back(poly.front());
  double acc = 0.0;      // arc length at start of segment i
  size_t i = 1;
  double seg = dist(poly[0], poly[1]);
  for (int k = 1; k < n; ++k) {
    const double target = k * ds;
    while (i < poly.size() - 1 && acc + seg < target) {
      acc += seg; ++i; seg = dist(poly[i - 1], poly[i]);
    }
    const double t = seg > 1e-12 ? (target - acc) / seg : 0.0;
    out.push_back({poly[i - 1].x + t * (poly[i].x - poly[i - 1].x), poly[i - 1].y + t * (poly[i].y - poly[i - 1].y)});
  }
  out.push_back(poly.back());
  return out;
}

namespace
{
inline double wrap(double a)
{
  while (a > M_PI) {a -= 2 * M_PI;}
  while (a < -M_PI) {a += 2 * M_PI;}
  return a;
}

inline double cellCost01(const GridView & g, double x, double y)
{
  int cx, cy;
  if (!g.worldToCell(x, y, cx, cy)) {return 1.0;}
  const uint8_t c = g.cost[g.idx(cx, cy)];
  return c == kCostUnknown ? 0.5 : c / 252.0;
}
}  // namespace

double scoreDense(const GridView & g, const std::vector<Vec2> & d, Vec2 start, const CostParams & cp)
{
  if (d.size() < 2) {return 0.0;}
  const double c0 = g.clearanceEst(start.x, start.y);
  const double d_des = cp.safe_radius + cp.desired_extra;
  double len = 0, smooth = 0, clear = 0, cst = 0;
  for (size_t i = 1; i < d.size(); ++i) {
    const double ds = dist(d[i - 1], d[i]);
    len += ds;
    const double cl = g.clearanceEst(d[i].x, d[i].y);
    double req = d_des;
    if (dist(d[i], start) <= cp.escape_dist) {req = std::min(req, std::max(0.0, c0));}
    clear += std::pow(std::max(0.0, req - cl), 2) * ds;
    cst += cellCost01(g, d[i].x, d[i].y) * ds;
    if (i + 1 < d.size()) {
      const double ds2 = dist(d[i], d[i + 1]);
      if (ds > 1e-9 && ds2 > 1e-9) {
        const double h1 = std::atan2(d[i].y - d[i - 1].y, d[i].x - d[i - 1].x);
        const double h2 = std::atan2(d[i + 1].y - d[i].y, d[i + 1].x - d[i].x);
        const double th = wrap(h2 - h1);
        smooth += th * th / (0.5 * (ds + ds2));
      }
    }
  }
  return cp.w_len * len + cp.w_smooth * smooth + cp.w_clear * clear + cp.w_cost * cst;
}

PathProblem::PathProblem(
  const GridView & g, const std::vector<Vec2> & seed, Vec2 start, Vec2 goal, const CostParams & cp,
  int n_interior, int n_samples)
: g_(g), cp_(cp), start_(start), goal_(goal), n_int_(n_interior), basis_(n_interior + 2, n_samples)
{
  seed_len_ = polylineLength(seed);
  // interior control points at equal arc-length fractions of the seed
  const std::vector<Vec2> dense = densify(seed, std::max(1e-3, seed_len_ / 2000.0));
  const double L = polylineLength(dense);
  base_.resize(n_int_);
  double acc = 0.0;
  size_t i = 1;
  for (int k = 0; k < n_int_; ++k) {
    const double target = L * (k + 1) / (n_int_ + 1);
    while (i < dense.size() - 1 && acc + dist(dense[i - 1], dense[i]) < target) {
      acc += dist(dense[i - 1], dense[i]); ++i;
    }
    const double seg = dist(dense[i - 1], dense[i]);
    const double t = seg > 1e-12 ? std::clamp((target - acc) / seg, 0.0, 1.0) : 0.0;
    base_[k] = {dense[i - 1].x + t * (dense[i].x - dense[i - 1].x), dense[i - 1].y + t * (dense[i].y - dense[i - 1].y)};
  }
  c0_est_ = g.clearanceEst(start.x, start.y);
}

void PathProblem::controlPoints(const double * x, double * cx, double * cy) const
{
  cx[0] = start_.x; cy[0] = start_.y;
  for (int k = 0; k < n_int_; ++k) {
    cx[k + 1] = base_[k].x + (x ? x[2 * k] : 0.0);
    cy[k + 1] = base_[k].y + (x ? x[2 * k + 1] : 0.0);
  }
  cx[n_int_ + 1] = goal_.x; cy[n_int_ + 1] = goal_.y;
}

Evaluation PathProblem::evaluate(const double * x) const
{
  double cx[40], cy[40];
  controlPoints(x, cx, cy);
  const double d_req = cp_.safe_radius + cp_.opt_margin;
  const double d_des = cp_.safe_radius + cp_.desired_extra;
  double len = 0, smooth = 0, clear = 0, cst = 0, viol = 0;
  double px = 0, py = 0;
  const int M = basis_.m;
  for (int s = 0; s < M; ++s) {
    const int j0 = basis_.j0[s];
    double X = 0, Y = 0, vx = 0, vy = 0, ax = 0, ay = 0;
    for (int q = 0; q < 4; ++q) {
      const double qx = cx[j0 + q], qy = cy[j0 + q];
      X += basis_.w0[s][q] * qx; Y += basis_.w0[s][q] * qy;
      vx += basis_.w1[s][q] * qx; vy += basis_.w1[s][q] * qy;
      ax += basis_.w2[s][q] * qx; ay += basis_.w2[s][q] * qy;
    }
    const double speed = std::hypot(vx, vy);
    const double ds = (s == 0) ? 0.0 : std::hypot(X - px, Y - py);
    px = X; py = Y;
    len += ds;
    const double wgt = (s == 0 || s == M - 1) ? 0.5 : 1.0;
    const double ds_param = speed * basis_.du * wgt;
    const double kappa = speed > 1e-6 ? std::abs(vx * ay - vy * ax) / (speed * speed * speed) : 1e3;
    smooth += std::min(kappa, 1e3) * std::min(kappa, 1e3) * ds_param;

    const double cl = g_.clearanceEst(X, Y);
    double req = d_req, des = d_des;
    if (std::hypot(X - start_.x, Y - start_.y) <= cp_.escape_dist) {
      req = std::min(req, std::max(0.0, c0_est_ - 0.03));
      des = std::min(des, std::max(0.0, c0_est_));
    }
    if (cl < req) {viol += (req - cl + 0.01) * std::max(ds, 1e-3);}
    clear += std::pow(std::max(0.0, des - cl), 2) * ds_param;
    int ccx, ccy;
    if (g_.worldToCell(X, Y, ccx, ccy)) {
      const uint8_t c = g_.cost[g_.idx(ccx, ccy)];
      cst += ((c == kCostUnknown) ? 0.5 : c / 252.0) * ds_param;
    }
  }
  Evaluation e;
  e.cost = cp_.w_len * len + cp_.w_smooth * smooth + cp_.w_clear * clear + cp_.w_cost * cst;
  e.violation = viol;
  return e;
}

std::vector<Vec2> PathProblem::decode(const double * x, double spacing) const
{
  double cx[40], cy[40];
  controlPoints(x, cx, cy);
  const int Mf = std::clamp(static_cast<int>(std::ceil(seed_len_ * 1.5 / (0.25 * spacing))), 100, 4000);
  BSplineBasis fine(n_int_ + 2, Mf);
  std::vector<Vec2> poly(Mf);
  for (int s = 0; s < Mf; ++s) {
    double X = 0, Y = 0;
    for (int q = 0; q < 4; ++q) {
      X += fine.w0[s][q] * cx[fine.j0[s] + q];
      Y += fine.w0[s][q] * cy[fine.j0[s] + q];
    }
    poly[s] = {X, Y};
  }
  poly.front() = start_;
  poly.back() = goal_;
  return densify(poly, spacing);
}

}  // namespace agrobot_emra_planner
