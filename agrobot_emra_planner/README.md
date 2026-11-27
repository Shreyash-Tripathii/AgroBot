# agrobot_emra_planner

Nav2 (Jazzy) global planner plugin based on the **Enhanced Mud Ring Algorithm** (Ali & Ameen, KJAR 10(2), 2025),
a hard collision-free guarantee, and event-driven replanning behavior trees.

## Enabling it
Off by default. `ros2 launch agrobot_navigation autonomy.launch.py use_emra:=true ...` layers
`agrobot_navigation/config/emra_overrides.yaml` (EMRA planner, NavFn backup, chassis footprint) and the event-driven BTs
on top of the untouched stock `nav2_params.yaml`. Without the flag the original NavFn + default BT setup runs.

## How it works
1. Copy the needed costmap region (under the costmap lock) and compute an exact distance transform.
2. Build a **guaranteed-valid seed**: clearance-aware A* + string pulling, re-verified by `PathValidator`.
3. Encode the path as displacements of ~N cubic B-spline control points from the seed (2N variables).
4. Run **EMRA** (Eqs. 1-10 of the paper) minimising length + curvature^2 + clearance deficit + cost, with Deb's
   feasibility rules.
5. Re-validate every candidate (EMRA leader, shrunken leaders, seed spline, seed polyline) with the exact
   square-vs-circle checker at 2.5 cm sampling; return the best *validated* one. If EMRA's result fails, the seed is
   returned, so the output is never unsafe and never worse than the seed.

`safe_radius` (0.17 m) = chassis half-diagonal (0.16) + 1 cm. Unknown cells are treated as walls unless
`allow_unknown: true`. Near the start the requirement is relaxed to the start's own clearance (`start_escape_dist`),
so the robot can drive away from a wall it is already touching.

### Deviations from / interpretations of the paper
* **V (Eq. 3) is not defined in the paper.** We use v <- clamp(K*v + r*(D_rand - D), +-vmax).
* The flowchart's `T_max < 0.3` is read as `t/T_max < 0.3`.
* Eq. 4 collapses toward the *origin* (this is why the paper's F1-F4 reach exactly 0). We therefore optimise
  *displacements from the seed path*, which puts the origin on a feasible path.
* Per-individual greedy selection and Deb's rules are added; `T_max` shrinks adaptively so the whole exploration ->
  exploitation schedule fits `max_planning_time` (important on a Raspberry Pi).

## Behavior trees (`behavior_trees/`)
Replan only when: goal updated, `IsPathValid` fails, or `IsPathDeviated` (robot > 0.6 m from the path). Checked at
2 Hz; the planner itself runs once per trigger. Falls back to `NavFnBackup` if EMRA fails (see caveat in the XML).
`autonomy.launch.py` injects both trees into `bt_navigator`.

## Benchmarks (60 random pairs per map, `ros2 run agrobot_emra_planner benchmark_planners.py`)
Run `ros2 launch agrobot_emra_planner planner_benchmark.launch.py map:=<yaml>` first.

Results (all 60 pairs solved by every planner; clearance measured on the raw map, chassis radius 0.16 m):

| Map | Planner | Length m | Bending (lower = smoother) | Min clearance m | Violations <0.16 m | Time s |
|---|---|---|---|---|---|---|
| bench_rows | **EMRA** | 21.84 | 1.50 | 0.181 | **0** | 0.24 |
| bench_rows | NavFn | 21.95 | 11.80 | 0.155 | 5 | 0.05 |
| bench_rows | Smac2D | 21.94 | 5.46 | 0.087 | 43 | 0.05 |
| bench_rows | ThetaStar | 21.78 | 8.64 | 0.133 | 32 | 0.05 |
| bench_yard | **EMRA** | 11.54 | 0.79 | 0.181 | **0** | 0.09 |
| bench_yard | NavFn | 11.52 | 13.42 | 0.155 | 3 | 0.02 |
| bench_yard | Smac2D | 11.52 | 5.51 | 0.087 | 43 | 0.02 |
| bench_yard | ThetaStar | 11.34 | 6.40 | 0.125 | 27 | 0.02 |

## Timing / memory (Raspberry Pi guidance)
Pinned to ONE core of a desktop CPU, 1 thread: mean 0.13 s, worst 0.22 s per plan on a 30x20 m static map and on
50x50 m rolling windows (0.10 and 0.05 m resolution). A Pi 4 is roughly 3-5x slower per core, so expect ~0.4-1.0 s;
`max_planning_time` (default 1.0 s) shrinks the iteration count so the schedule always fits. A 1000x1000 window needs
~15 MB. Not measured on real Pi hardware.

## Known limits
* The guarantee is for the *planned* path. The controller (DWB) and localisation add tracking error: in simulation
  executed paths came within 0.116 m of walls at worst, vs 0.17 m planned. The local costmap uses the chassis polygon
  for this reason.
* `NavFnBackup` in the BT only guarantees the costmap's inscribed radius.
* Goals inside / within `safe_radius` of a wall are moved up to `goal_tolerance` away; if none is safe, the plan fails.
