#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <random>

#include "agrobot_emra_planner/emra_optimizer.hpp"
#include "agrobot_emra_planner/grid_map_view.hpp"
#include "agrobot_emra_planner/path_validator.hpp"
#include "agrobot_emra_planner/planning_core.hpp"

using namespace agrobot_emra_planner;

namespace
{

GridView makeGrid(int w, int h, double res = 0.05, bool allow_unknown = false)
{
  GridView g;
  g.init(w, h, res, -0.5 * w * res, -0.5 * h * res, allow_unknown);
  return g;
}

void fillRect(GridView & g, double x0, double y0, double x1, double y1, uint8_t c = kCostLethal)
{
  for (int y = 0; y < g.h; ++y) {
    for (int x = 0; x < g.w; ++x) {
      const Vec2 p = g.cellCenter(x, y);
      if (p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1) {g.cost[g.idx(x, y)] = c;}
    }
  }
}

// Independent brute-force checker: no EDT shortcuts, scans every blocked cell.
double bruteClearance(const GridView & g, Vec2 p)
{
  double best = std::min({p.x - g.ox, g.ox + g.w * g.res - p.x, p.y - g.oy, g.oy + g.h * g.res - p.y});
  for (int y = 0; y < g.h; ++y) {
    for (int x = 0; x < g.w; ++x) {
      if (!g.blockedCell(x, y)) {continue;}
      const double rx0 = g.ox + x * g.res, ry0 = g.oy + y * g.res;
      const double dx = std::max({rx0 - p.x, 0.0, p.x - rx0 - g.res});
      const double dy = std::max({ry0 - p.y, 0.0, p.y - ry0 - g.res});
      best = std::min(best, std::hypot(dx, dy));
    }
  }
  return best;
}

double pathMinClearance(const GridView & g, const std::vector<Vec2> & path, Vec2 start, double escape)
{
  double best = 1e9;
  for (size_t i = 1; i < path.size(); ++i) {
    const double len = dist(path[i - 1], path[i]);
    const int n = std::max(1, static_cast<int>(len / 0.01));
    for (int k = 0; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      const Vec2 q{path[i - 1].x + t * (path[i].x - path[i - 1].x), path[i - 1].y + t * (path[i].y - path[i - 1].y)};
      if (dist(q, start) <= escape) {continue;}
      best = std::min(best, bruteClearance(g, q));
    }
  }
  return best;
}

PlanParams testParams()
{
  PlanParams p;
  p.emra.adaptive_iterations = false;
  p.emra.seed = 12345;
  p.emra.max_time_s = 10.0;
  return p;
}

}  // namespace

TEST(Grid, EdtMatchesBruteForce)
{
  GridView g = makeGrid(60, 50);
  std::mt19937 rng(1);
  for (auto & c : g.cost) {c = (rng() % 12 == 0) ? kCostLethal : 0;}
  g.computeEdt();
  for (int y = 0; y < g.h; ++y) {
    for (int x = 0; x < g.w; ++x) {
      double best = 1e9;
      const Vec2 c = g.cellCenter(x, y);
      // border ring is blocked
      best = std::min({static_cast<double>(x + 1), static_cast<double>(g.w - x), static_cast<double>(y + 1),
          static_cast<double>(g.h - y)}) * g.res;
      for (int yy = 0; yy < g.h; ++yy) {
        for (int xx = 0; xx < g.w; ++xx) {
          if (g.blockedCell(xx, yy)) {best = std::min(best, dist(c, g.cellCenter(xx, yy)));}
        }
      }
      ASSERT_NEAR(g.edt[g.idx(x, y)], best, 1e-4) << x << "," << y;
    }
  }
}

TEST(Grid, ExactClearanceMatchesBruteForce)
{
  GridView g = makeGrid(80, 80);
  std::mt19937 rng(2);
  for (auto & c : g.cost) {c = (rng() % 15 == 0) ? kCostLethal : 0;}
  g.computeEdt();
  std::uniform_real_distribution<double> U(-1.9, 1.9);
  for (int i = 0; i < 500; ++i) {
    const Vec2 p{U(rng), U(rng)};
    const double cap = 0.4;
    ASSERT_NEAR(g.exactClearance(p.x, p.y, cap), std::min(cap, bruteClearance(g, p)), 1e-9);
  }
}

TEST(Emra, SphereAtOriginConverges)
{
  const int D = 10;
  std::vector<double> lb(D, -5), ub(D, 5);
  EmraParams p;
  p.adaptive_iterations = false; p.max_iterations = 200; p.seed = 7; p.max_time_s = 10;
  auto f = [&](const double * x) {
      Evaluation e;
      for (int i = 0; i < D; ++i) {e.cost += x[i] * x[i];}
      return e;
    };
  const auto r = emraOptimize(f, lb, ub, p, {}, nullptr);
  EXPECT_LT(r.eval.cost, 1e-3);
}

TEST(Emra, DeterministicAndRespectsBounds)
{
  const int D = 6;
  std::vector<double> lb(D, -1), ub(D, 2);
  EmraParams p;
  p.adaptive_iterations = false; p.max_iterations = 60; p.seed = 99; p.max_time_s = 10;
  auto f = [&](const double * x) {
      Evaluation e;
      for (int i = 0; i < D; ++i) {e.cost += (x[i] - 0.7) * (x[i] - 0.7);}
      return e;
    };
  const auto a = emraOptimize(f, lb, ub, p, {}, nullptr);
  const auto b = emraOptimize(f, lb, ub, p, {}, nullptr);
  EXPECT_EQ(a.best, b.best);
  for (int i = 0; i < D; ++i) {EXPECT_GE(a.best[i], -1.0); EXPECT_LE(a.best[i], 2.0);}
}

TEST(Emra, FeasibleBeatsInfeasible)
{
  EXPECT_TRUE(better({5.0, 0.0}, {1.0, 0.1}));
  EXPECT_FALSE(better({1.0, 0.1}, {5.0, 0.0}));
  EXPECT_TRUE(better({1.0, 0.1}, {0.5, 0.2}));
}

TEST(Plan, EmptySpaceStraightish)
{
  GridView g = makeGrid(240, 240);
  g.computeEdt();
  const auto r = planPath(g, {-4, -4}, {4, 4}, testParams(), nullptr);
  EXPECT_NEAR(r.stats.final_length, std::hypot(8, 8), 0.1);
  EXPECT_GE(pathMinClearance(g, r.path, {-4, -4}, 0.5), 0.17 - 1e-6);
}

TEST(Plan, CorridorAndDoor)
{
  GridView g = makeGrid(240, 240);
  fillRect(g, -0.1, -6, 0.1, -0.35);       // wall with a 0.7 m door centred at y=0
  fillRect(g, -0.1, 0.35, 0.1, 6);
  g.computeEdt();
  const auto r = planPath(g, {-3, 0.0}, {3, 0.0}, testParams(), nullptr);
  EXPECT_GE(pathMinClearance(g, r.path, {-3, 0}, 0.5), 0.17 - 1e-6);
  EXPECT_NEAR(r.path.back().x, 3.0, 1e-6);
}

TEST(Plan, DoorTooNarrowFails)
{
  GridView g = makeGrid(240, 240);
  fillRect(g, -0.1, -6, 0.1, -0.15);       // 0.3 m gap < 2*0.17
  fillRect(g, -0.1, 0.15, 0.1, 6);
  g.computeEdt();
  EXPECT_THROW(planPath(g, {-3, 0}, {3, 0}, testParams(), nullptr), PlanException);
}

TEST(Plan, UTrapDetour)
{
  GridView g = makeGrid(240, 240);
  fillRect(g, -2, -2, -1.8, 2);            // U opening to the left, goal inside
  fillRect(g, -2, 1.8, 2, 2);
  fillRect(g, -2, -2, 2, -1.8);
  g.computeEdt();
  const auto r = planPath(g, {1.0, 0.0}, {-4, 0}, testParams(), nullptr);
  EXPECT_GE(pathMinClearance(g, r.path, {1, 0}, 0.5), 0.17 - 1e-6);
  EXPECT_GT(r.stats.final_length, 5.0);
}

TEST(Plan, UnreachableAndOccupiedGoal)
{
  GridView g = makeGrid(240, 240);
  fillRect(g, -6, -0.1, 6, 0.1);           // complete wall
  g.computeEdt();
  EXPECT_THROW(planPath(g, {0, -2}, {0, 2}, testParams(), nullptr), PlanException);
  GridView g2 = makeGrid(240, 240);
  fillRect(g2, 1, -1, 3, 1);
  g2.computeEdt();
  try {
    planPath(g2, {-2, 0}, {2, 0}, testParams(), nullptr);
    FAIL();
  } catch (const PlanException & e) {EXPECT_EQ(e.code, PlanError::kGoalOccupied);}
}

TEST(Plan, StartNextToWallEscapes)
{
  GridView g = makeGrid(240, 240);
  fillRect(g, -6, -0.5, 6, -0.3);          // wall; robot 0.09 m above its top face
  g.computeEdt();
  const Vec2 s{0, -0.2};
  const auto r = planPath(g, s, {3, 3}, testParams(), nullptr);
  EXPECT_GE(pathMinClearance(g, r.path, s, 0.5), 0.17 - 1e-6);
}

TEST(Plan, StartEqualsGoal)
{
  GridView g = makeGrid(100, 100);
  g.computeEdt();
  const auto r = planPath(g, {0, 0}, {0.01, 0}, testParams(), nullptr);
  EXPECT_EQ(r.path.size(), 2u);
}

// The hard guarantee, checked by the independent brute-force checker on random maps.
TEST(Plan, RandomMapsNeverViolateClearance)
{
  std::mt19937 rng(2024);
  std::uniform_real_distribution<double> U(-4.5, 4.5);
  int planned = 0, failed = 0;
  for (int m = 0; m < 12; ++m) {
    GridView g = makeGrid(200, 200);
    for (int k = 0; k < 14; ++k) {
      const double cx = U(rng), cy = U(rng), hw = 0.1 + 0.5 * std::abs(U(rng)) / 4.5, hh = 0.1 + 0.5 * std::abs(U(rng)) / 4.5;
      fillRect(g, cx - hw, cy - hh, cx + hw, cy + hh);
    }
    g.computeEdt();
    for (int q = 0; q < 12; ++q) {
      const Vec2 s{U(rng), U(rng)}, e{U(rng), U(rng)};
      PlanParams p = testParams();
      p.emra.max_iterations = 40;
      try {
        PathValidator v(g, 0.17, 0.5, s);
        if (!v.pointOk(s)) {continue;}
        const auto r = planPath(g, s, e, p, nullptr);
        ++planned;
        ASSERT_GE(pathMinClearance(g, r.path, s, 0.5), 0.17 - 1e-6) << "map " << m << " query " << q << " src " << r.stats.source;
        ASSERT_NEAR(r.path.front().x, s.x, 1e-9);
      } catch (const PlanException &) {++failed;}
    }
  }
  EXPECT_GT(planned, 60);
  RecordProperty("planned", planned);
  RecordProperty("failed", failed);
}

TEST(Plan, WithinTimeBudget)
{
  GridView g = makeGrid(600, 600);
  for (int k = 0; k < 40; ++k) {fillRect(g, -12 + k * 0.6, -3, -11.9 + k * 0.6, 3);}
  g.computeEdt();
  PlanParams p;
  p.emra.max_time_s = 1.0;
  const auto t0 = std::chrono::steady_clock::now();
  const auto r = planPath(g, {-14, -10}, {14, 10}, p, nullptr);
  const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_LT(dt, 2.0);
  EXPECT_GE(pathMinClearance(g, r.path, {-14, -10}, 0.5), 0.17 - 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
