#pragma once
// Enhanced Mud Ring Algorithm (Ali & Ameen, KJAR 10(2), 2025) - generic, ROS-free.
//
// Faithful to Algorithm 2 / Fig. 1 / Eqs. 1-10 of the paper. Items the paper leaves open
// (documented in the README): the velocity V in Eq. 3, constraint handling beyond the Eq. 10
// penalty (we use Deb's feasibility rules), and per-individual greedy selection.
#include <cstdint>
#include <functional>
#include <vector>

namespace agrobot_emra_planner
{

struct Evaluation
{
  double cost{0.0};        // objective (minimise)
  double violation{0.0};   // 0 == feasible
};

// Deb's rules: feasible beats infeasible; two feasible -> lower cost; two infeasible -> lower violation.
inline bool better(const Evaluation & a, const Evaluation & b)
{
  const bool fa = a.violation <= 1e-9, fb = b.violation <= 1e-9;
  if (fa && fb) {return a.cost < b.cost;}
  if (fa != fb) {return fa;}
  return a.violation < b.violation;
}

struct EmraParams
{
  int pop_size{30};
  int max_iterations{150};     // T_max
  int stall_iterations{40};    // early stop when the leader has not improved
  double max_time_s{1.0};      // hard wall-clock budget
  bool adaptive_iterations{true};  // shrink T_max so the whole schedule fits max_time_s
  int min_iterations{20};
  int num_threads{1};
  uint64_t seed{0};            // 0 -> random
  double v_max_frac{0.2};      // |V| <= v_max_frac * (ub - lb)
  double init_spread{0.4};     // initial population spread as a fraction of the bounds
  bool greedy_selection{true}; // keep an individual's new position only if it is not worse
  double bound_penalty_weight{1.0};  // weight of Eq. 10
};

struct EmraResult
{
  std::vector<double> best;
  Evaluation eval;
  int iterations{0};
  int evaluations{0};
  double elapsed_s{0.0};
  bool cancelled{false};
  std::vector<double> history;   // leader cost per iteration
};

using ObjectiveFn = std::function<Evaluation(const double *)>;

// `seeds` are inserted verbatim as the first individuals (e.g. the zero-displacement path).
EmraResult emraOptimize(
  const ObjectiveFn & f, const std::vector<double> & lb, const std::vector<double> & ub,
  const EmraParams & p, const std::vector<std::vector<double>> & seeds,
  const std::function<bool()> & cancel);

}  // namespace agrobot_emra_planner
