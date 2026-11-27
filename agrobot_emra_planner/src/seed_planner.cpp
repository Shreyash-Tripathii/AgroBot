#include "agrobot_emra_planner/seed_planner.hpp"

#include <chrono>
#include <limits>
#include <queue>

namespace agrobot_emra_planner
{
namespace
{

struct AStarOut
{
  bool ok{false};
  bool cancelled{false};
  std::vector<Vec2> chain;
  int expansions{0};
};

AStarOut astar(
  const GridView & g, Vec2 start, Vec2 goal, double r_allow, const SeedParams & p,
  const std::function<bool()> & cancel)
{
  AStarOut out;
  int sx, sy, gx, gy;
  if (!g.worldToCell(start.x, start.y, sx, sy) || !g.worldToCell(goal.x, goal.y, gx, gy)) {return out;}
  const double e_start = g.edt[g.idx(sx, sy)];
  const double e_floor = std::min(r_allow, e_start - 1e-6);

  auto allowed = [&](int x, int y) -> bool {
      if (g.blockedCell(x, y)) {return false;}
      const double e = g.edt[g.idx(x, y)];
      if (e >= r_allow) {return true;}
      const Vec2 c = g.cellCenter(x, y);
      return dist(c, start) <= p.escape_dist + g.res && e >= e_floor;
    };
  if (!allowed(sx, sy) || !allowed(gx, gy)) {return out;}

  const int n = g.w * g.h;
  const float INF = std::numeric_limits<float>::infinity();
  std::vector<float> gs(n, INF);
  std::vector<int> parent(n, -1);
  using QE = std::pair<float, int>;
  std::priority_queue<QE, std::vector<QE>, std::greater<QE>> open;

  const double d_pref = p.pref_extra;
  auto mult = [&](int x, int y) -> double {
      const int i = g.idx(x, y);
      const double e = g.edt[i];
      const double pen = std::clamp((p.safe_radius + d_pref - e) / d_pref, 0.0, 1.0);
      const uint8_t c = g.cost[i];
      const double cc = (c == kCostUnknown) ? 0.5 : (c / 252.0);
      return 1.0 + p.w_prox * pen * pen + p.w_cost * cc + (c == kCostUnknown ? 1.0 : 0.0);
    };
  auto heur = [&](int x, int y) -> float {
      const double dx = std::abs(x - gx), dy = std::abs(y - gy);
      return static_cast<float>(g.res * (std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy)));
    };

  const int s = g.idx(sx, sy), t = g.idx(gx, gy);
  gs[s] = 0.0f;
  open.push({heur(sx, sy), s});
  static const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const auto t0 = std::chrono::steady_clock::now();
  bool found = false;
  while (!open.empty()) {
    const auto [f, cur] = open.top();
    open.pop();
    const int cx = cur % g.w, cy = cur / g.w;
    if (f > gs[cur] + heur(cx, cy) + 1e-4f) {continue;}   // stale entry
    if (cur == t) {found = true; break;}
    if ((++out.expansions & 2047) == 0) {
      if (cancel && cancel()) {out.cancelled = true; return out;}
      const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (el > p.timeout_s) {return out;}
    }
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + dxs[k], ny = cy + dys[k];
      if (!g.inside(nx, ny) || !allowed(nx, ny)) {continue;}
      if (k >= 4 && (!allowed(cx + dxs[k], cy) || !allowed(cx, cy + dys[k]))) {continue;}  // no corner cutting
      const double len = (k >= 4 ? 1.41421356 : 1.0) * g.res;
      const float ng = gs[cur] + static_cast<float>(len * mult(nx, ny));
      const int ni = g.idx(nx, ny);
      if (ng < gs[ni]) {
        gs[ni] = ng; parent[ni] = cur;
        open.push({ng + heur(nx, ny), ni});
      }
    }
  }
  if (!found) {return out;}
  std::vector<Vec2> rev;
  for (int c = t; c != -1; c = parent[c]) {rev.push_back(g.cellCenter(c % g.w, c / g.w));}
  out.chain.assign(rev.rbegin(), rev.rend());
  out.chain.front() = start;
  if (out.chain.size() == 1) {out.chain.push_back(goal);} else {out.chain.back() = goal;}
  out.ok = true;
  return out;
}

std::vector<Vec2> stringPull(const std::vector<Vec2> & chain, const PathValidator & v)
{
  std::vector<Vec2> out{chain.front()};
  const int last = static_cast<int>(chain.size()) - 1;
  int i = 0;
  while (i < last) {
    int next = i + 1;
    for (int j = last; j > i + 1; --j) {
      if (v.segmentOk(chain[i], chain[j])) {next = j; break;}
    }
    out.push_back(chain[next]);
    i = next;
  }
  return out;
}

}  // namespace

SeedResult planSeed(
  const GridView & g, Vec2 start, Vec2 goal, const SeedParams & p,
  const std::function<bool()> & cancel)
{
  SeedResult res;
  const PathValidator truth(g, p.safe_radius, p.escape_dist, start);

  // Attempt 1: standard conservatism. Attempt 2: provably-safe (edt >= safe + 1.5 cells).
  const double r_allow[2] = {p.safe_radius + 0.75 * g.res, p.safe_radius + 1.5 * g.res};
  for (int attempt = 0; attempt < 2; ++attempt) {
    AStarOut a = astar(g, start, goal, r_allow[attempt], p, cancel);
    res.expansions += a.expansions;
    if (a.cancelled) {res.cancelled = true; return res;}
    if (!a.ok) {continue;}
    res.chain = a.chain;
    for (double m : p.margins) {
      const PathValidator pv(g, p.safe_radius + m, p.escape_dist, start);
      std::vector<Vec2> pulled = stringPull(a.chain, pv);
      if (truth.checkPolyline(pulled).ok) {
        res.ok = true; res.poly = std::move(pulled); res.margin_used = m;
        return res;
      }
    }
    if (truth.checkPolyline(a.chain).ok) {   // un-pulled chain is the last resort
      res.ok = true; res.poly = a.chain; res.margin_used = 0.0;
      return res;
    }
  }
  return res;
}

}  // namespace agrobot_emra_planner
