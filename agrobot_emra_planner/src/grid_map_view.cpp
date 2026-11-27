#include "agrobot_emra_planner/grid_map_view.hpp"

namespace agrobot_emra_planner
{

void GridView::init(int width, int height, double resolution, double origin_x, double origin_y, bool allow_unk)
{
  w = width; h = height; res = resolution; ox = origin_x; oy = origin_y; allow_unknown = allow_unk;
  cost.assign(static_cast<size_t>(w) * h, 0);
  edt.assign(static_cast<size_t>(w) * h, 0.0f);
}

bool GridView::worldToCell(double wx, double wy, int & cx, int & cy) const
{
  const double fx = (wx - ox) / res, fy = (wy - oy) / res;
  if (fx < 0.0 || fy < 0.0) {return false;}
  cx = static_cast<int>(fx); cy = static_cast<int>(fy);
  return cx < w && cy < h;
}

bool GridView::blockedCell(int x, int y) const
{
  if (!inside(x, y)) {return true;}
  const uint8_t c = cost[idx(x, y)];
  return c == kCostLethal || (c == kCostUnknown && !allow_unknown);
}

// Felzenszwalb & Huttenlocher squared-distance transform, one dimension.
static void edt1d(const double * f, int n, double * d, int * v, double * z)
{
  int k = 0;
  v[0] = 0; z[0] = -1e30; z[1] = 1e30;
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + static_cast<double>(q) * q) - (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
      (2.0 * q - 2.0 * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q] + static_cast<double>(q) * q) - (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
        (2.0 * q - 2.0 * v[k]);
    }
    ++k; v[k] = q; z[k] = s; z[k + 1] = 1e30;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {++k;}
    d[q] = static_cast<double>(q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

void GridView::computeEdt()
{
  // Pad by one cell of "blocked" so the view border counts as an obstacle.
  const int W = w + 2, H = h + 2;
  const double INF = 1e12;
  std::vector<double> g(static_cast<size_t>(W) * H);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const bool border = (x == 0 || y == 0 || x == W - 1 || y == H - 1);
      g[static_cast<size_t>(y) * W + x] = (border || blockedCell(x - 1, y - 1)) ? 0.0 : INF;
    }
  }
  const int m = std::max(W, H);
  std::vector<double> f(m), d(m), z(m + 1);
  std::vector<int> v(m);
  for (int x = 0; x < W; ++x) {
    for (int y = 0; y < H; ++y) {f[y] = g[static_cast<size_t>(y) * W + x];}
    edt1d(f.data(), H, d.data(), v.data(), z.data());
    for (int y = 0; y < H; ++y) {g[static_cast<size_t>(y) * W + x] = d[y];}
  }
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {f[x] = g[static_cast<size_t>(y) * W + x];}
    edt1d(f.data(), W, d.data(), v.data(), z.data());
    for (int x = 0; x < W; ++x) {g[static_cast<size_t>(y) * W + x] = d[x];}
  }
  edt.resize(static_cast<size_t>(w) * h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      edt[idx(x, y)] = static_cast<float>(std::sqrt(g[static_cast<size_t>(y + 1) * W + (x + 1)]) * res);
    }
  }
}

double GridView::clearanceEst(double px, double py) const
{
  if (px < ox || py < oy || px >= ox + w * res || py >= oy + h * res) {return -1.0;}
  const double fx = (px - ox) / res - 0.5, fy = (py - oy) / res - 0.5;
  const int x0 = static_cast<int>(std::floor(fx)), y0 = static_cast<int>(std::floor(fy));
  const double tx = fx - x0, ty = fy - y0;
  const int xa = std::clamp(x0, 0, w - 1), xb = std::clamp(x0 + 1, 0, w - 1);
  const int ya = std::clamp(y0, 0, h - 1), yb = std::clamp(y0 + 1, 0, h - 1);
  const double v00 = edt[idx(xa, ya)], v10 = edt[idx(xb, ya)];
  const double v01 = edt[idx(xa, yb)], v11 = edt[idx(xb, yb)];
  const double v = (v00 * (1 - tx) + v10 * tx) * (1 - ty) + (v01 * (1 - tx) + v11 * tx) * ty;
  return v - 0.5 * res;
}

double GridView::exactClearance(double px, double py, double cap) const
{
  if (px < ox || py < oy || px >= ox + w * res || py >= oy + h * res) {return 0.0;}
  double best = std::min({px - ox, ox + w * res - px, py - oy, oy + h * res - py});
  best = std::min(best, cap);
  const int cx = std::clamp(static_cast<int>((px - ox) / res), 0, w - 1);
  const int cy = std::clamp(static_cast<int>((py - oy) / res), 0, h - 1);
  // Rigorous lower bound on the true distance: edt - 2*(half diagonal of a cell).
  if (edt[idx(cx, cy)] - 1.4143 * res >= cap) {return best;}
  const int x0 = std::max(0, static_cast<int>(std::floor((px - cap - ox) / res)));
  const int x1 = std::min(w - 1, static_cast<int>(std::floor((px + cap - ox) / res)));
  const int y0 = std::max(0, static_cast<int>(std::floor((py - cap - oy) / res)));
  const int y1 = std::min(h - 1, static_cast<int>(std::floor((py + cap - oy) / res)));
  for (int y = y0; y <= y1; ++y) {
    const double ry0 = oy + y * res, ry1 = ry0 + res;
    const double dy = std::max({ry0 - py, 0.0, py - ry1});
    for (int x = x0; x <= x1; ++x) {
      if (!blockedCell(x, y)) {continue;}
      const double rx0 = ox + x * res, rx1 = rx0 + res;
      const double dx = std::max({rx0 - px, 0.0, px - rx1});
      const double d = std::hypot(dx, dy);
      if (d < best) {best = d;}
    }
  }
  return best;
}

}  // namespace agrobot_emra_planner
