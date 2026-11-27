#include "agrobot_emra_planner/emra_optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <thread>

namespace agrobot_emra_planner
{

EmraResult emraOptimize(
  const ObjectiveFn & f, const std::vector<double> & lb, const std::vector<double> & ub,
  const EmraParams & p, const std::vector<std::vector<double>> & seeds,
  const std::function<bool()> & cancel)
{
  using clock = std::chrono::steady_clock;
  const auto t_start = clock::now();
  auto elapsed = [&]() {return std::chrono::duration<double>(clock::now() - t_start).count();};

  const int dim = static_cast<int>(lb.size());
  const int P = std::max(4, p.pop_size);
  std::mt19937_64 rng(p.seed ? p.seed : std::random_device{}());
  std::uniform_real_distribution<double> U(0.0, 1.0);
  std::normal_distribution<double> N01(0.0, 1.0);

  std::vector<std::vector<double>> X(P, std::vector<double>(dim)), V(P, std::vector<double>(dim));
  std::vector<double> vmax(dim);
  for (int j = 0; j < dim; ++j) {vmax[j] = p.v_max_frac * (ub[j] - lb[j]);}

  for (int i = 0; i < P; ++i) {
    for (int j = 0; j < dim; ++j) {
      const double c = 0.5 * (lb[j] + ub[j]), hw = 0.5 * (ub[j] - lb[j]) * p.init_spread;
      X[i][j] = (i < static_cast<int>(seeds.size())) ? seeds[i][j] : c + (2.0 * U(rng) - 1.0) * hw;
      V[i][j] = (2.0 * U(rng) - 1.0) * vmax[j];
    }
  }

  EmraResult R;
  std::vector<Evaluation> E(P);

  auto evalBatch = [&](const std::vector<std::vector<double>> & pts, std::vector<Evaluation> & out) {
      const int n = static_cast<int>(pts.size());
      const int T = std::max(1, std::min(p.num_threads, n));
      if (T == 1) {
        for (int i = 0; i < n; ++i) {out[i] = f(pts[i].data());}
      } else {
        std::vector<std::thread> th;
        auto work = [&](int a, int b) {for (int i = a; i < b; ++i) {out[i] = f(pts[i].data());}};
        const int chunk = (n + T - 1) / T;
        for (int k = 1; k < T; ++k) {th.emplace_back(work, k * chunk, std::min(n, (k + 1) * chunk));}
        work(0, std::min(n, chunk));
        for (auto & t : th) {t.join();}
      }
      R.evaluations += n;
    };

  const auto t_eval0 = clock::now();
  evalBatch(X, E);
  const double t_pop = std::chrono::duration<double>(clock::now() - t_eval0).count();

  int best_i = 0;
  for (int i = 1; i < P; ++i) {if (better(E[i], E[best_i])) {best_i = i;}}
  std::vector<double> leader = X[best_i];
  Evaluation leader_e = E[best_i];

  int Tmax = p.max_iterations;
  if (p.adaptive_iterations && t_pop > 0.0) {
    const int fit = static_cast<int>((p.max_time_s - elapsed()) / (1.25 * t_pop));
    Tmax = std::clamp(fit, p.min_iterations, p.max_iterations);
  }

  std::vector<std::vector<double>> Xn(P, std::vector<double>(dim));
  std::vector<double> pen(P);
  std::vector<Evaluation> En(P);
  int stall = 0;
  const double TWO_PI = 6.283185307179586;

  int t = 0;
  for (; t < Tmax; ++t) {
    if (cancel && cancel()) {R.cancelled = true; break;}
    if (elapsed() > p.max_time_s) {break;}
    const double tt = static_cast<double>(t) / Tmax;
    const double a = 2.0 * (1.0 - tt);                       // Eq. 2

    for (int i = 0; i < P; ++i) {
      const double l = U(rng);
      int rnd = static_cast<int>(U(rng) * P) % P;
      if (rnd == i) {rnd = (rnd + 1) % P;}
      double bp = 0.0;
      for (int j = 0; j < dim; ++j) {
        const double r = U(rng);
        double K;
        if (tt < 0.3) {                                      // Eq. 7  (early exploration)
          K = (2.0 * a * r - a) * (1.0 - tt) + 0.1 * N01(rng);
        } else if (tt < 0.7) {                               // Eq. 8  (balanced)
          K = (2.0 * a * r - a) * std::exp(-tt / 10.0);
        } else {                                             // Eq. 9  (exploitation)
          K = 0.5 * (2.0 * a * r - a);
        }
        K = std::clamp(K, -2.0, 2.0);
        const double C = 2.0 * U(rng);                       // Eq. 6
        double nv;
        if (std::abs(K) >= 1.0) {                            // exploration: Eq. 3, D = D + V
          double v = K * V[i][j] + U(rng) * (X[rnd][j] - X[i][j]);
          v = std::clamp(v, -vmax[j], vmax[j]);
          V[i][j] = v;
          nv = X[i][j] + v;
        } else {                                             // exploitation: Eqs. 4-5
          const double A = std::abs(C * leader[j] - X[i][j]);
          nv = leader[j] * std::sin(TWO_PI * l) - K * A;
        }
        // Eq. 10 bound penalty (pre-clamp), then clamp (Algorithm 2 line 21)
        bp += std::pow(std::max(0.0, nv - ub[j]), 2) + std::pow(std::max(0.0, lb[j] - nv), 2);
        Xn[i][j] = std::clamp(nv, lb[j], ub[j]);
      }
      pen[i] = bp;
    }
    evalBatch(Xn, En);
    for (int i = 0; i < P; ++i) {
      En[i].cost += p.bound_penalty_weight * pen[i];
      if (!p.greedy_selection || !better(E[i], En[i])) {
        X[i] = Xn[i]; E[i] = En[i];
      }
      if (better(E[i], leader_e)) {
        leader = X[i]; leader_e = E[i]; stall = -1;
      }
    }
    ++stall;
    R.history.push_back(leader_e.cost);
    if (stall >= p.stall_iterations) {++t; break;}
  }

  R.best = leader;
  R.eval = leader_e;
  R.iterations = t;
  R.elapsed_s = elapsed();
  return R;
}

}  // namespace agrobot_emra_planner
