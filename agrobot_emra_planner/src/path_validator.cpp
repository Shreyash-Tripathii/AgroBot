#include "agrobot_emra_planner/path_validator.hpp"

namespace agrobot_emra_planner
{

PathValidator::PathValidator(const GridView & g, double required, double escape_dist, Vec2 start)
: g_(g), required_(required), escape_(escape_dist), start_(start)
{
  step_ = std::min(0.5 * g.res, 0.025);
  c0_ = g.exactClearance(start.x, start.y, 1.0);
}

bool PathValidator::pointOk(const Vec2 & p) const
{
  int cx, cy;
  if (!g_.worldToCell(p.x, p.y, cx, cy)) {return false;}
  if (g_.blockedCell(cx, cy)) {return false;}
  const uint8_t c = g_.cost[g_.idx(cx, cy)];
  double req = required_ + 0.5 * step_;   // half-step margin covers the gap between samples
  if (dist(p, start_) <= escape_) {
    req = std::min(req, std::max(0.0, c0_ - 1e-3));
  } else if (c >= kCostInscribed && c != kCostUnknown) {
    return false;
  }
  return g_.exactClearance(p.x, p.y, req) >= req;
}

bool PathValidator::segmentOk(const Vec2 & a, const Vec2 & b) const
{
  const double len = dist(a, b);
  const int n = std::max(1, static_cast<int>(std::ceil(len / step_)));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    if (!pointOk({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)})) {return false;}
  }
  return true;
}

PathValidator::Report PathValidator::checkPolyline(const std::vector<Vec2> & pts) const
{
  Report r;
  if (pts.empty()) {r.ok = false; return r;}
  if (!pointOk(pts[0])) {r.ok = false; r.first_bad = 0; return r;}
  for (size_t i = 1; i < pts.size(); ++i) {
    if (!segmentOk(pts[i - 1], pts[i])) {r.ok = false; r.first_bad = static_cast<int>(i); return r;}
  }
  return r;
}

double PathValidator::minClearance(const std::vector<Vec2> & pts, double cap) const
{
  double best = cap;
  for (size_t i = 0; i < pts.size(); ++i) {
    const double len = (i == 0) ? 0.0 : dist(pts[i - 1], pts[i]);
    const int n = std::max(1, static_cast<int>(std::ceil(len / step_)));
    for (int k = (i == 0 ? 0 : 1); k <= (i == 0 ? 0 : n); ++k) {
      const double t = static_cast<double>(k) / n;
      Vec2 q = pts[i];
      if (i > 0) {q = {pts[i - 1].x + t * (pts[i].x - pts[i - 1].x), pts[i - 1].y + t * (pts[i].y - pts[i - 1].y)};}
      best = std::min(best, g_.exactClearance(q.x, q.y, cap));
    }
  }
  return best;
}

}  // namespace agrobot_emra_planner
