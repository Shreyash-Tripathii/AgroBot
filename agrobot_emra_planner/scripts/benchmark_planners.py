#!/usr/bin/env python3
"""Compare global planners on random start/goal pairs through planner_server's ComputePathToPose.

Independent of every planner: safety/clearance is measured on the raw map image with an exact
distance transform, not on the costmap the planners saw.
"""
import argparse
import csv
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from PIL import Image
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy import ndimage

CHASSIS_R = 0.16   # circumscribed radius of the robot body
ROBOT_R = 0.11     # radius the costmap believes in


class Map:
    def __init__(self, yaml_path):
        import os
        d = yaml.safe_load(open(yaml_path))
        img = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), d['image'])).convert('L'))
        self.res = d['resolution']
        self.ox, self.oy = d['origin'][0], d['origin'][1]
        img = np.flipud(img)
        self.occ = img < 0.35 * 255          # occupied
        self.unk = (img > 0.65 * 255 * 0.76) & (img < 250)   # trinary 205 -> unknown
        free = ~self.occ & ~self.unk
        self.edt = ndimage.distance_transform_edt(~self.occ) * self.res   # to nearest occupied
        self.edt_blk = ndimage.distance_transform_edt(~(self.occ | self.unk)) * self.res
        self.free = free

    def clearance(self, x, y):
        i = int((y - self.oy) / self.res); j = int((x - self.ox) / self.res)
        if i < 0 or j < 0 or i >= self.occ.shape[0] or j >= self.occ.shape[1]:
            return 0.0
        return float(self.edt[i, j]) - 0.5 * self.res

    def is_unknown(self, x, y):
        i = int((y - self.oy) / self.res); j = int((x - self.ox) / self.res)
        if i < 0 or j < 0 or i >= self.occ.shape[0] or j >= self.occ.shape[1]:
            return True
        return bool(self.unk[i, j])


def resample(pts, step):
    pts = np.asarray(pts)
    d = np.hypot(*np.diff(pts, axis=0).T)
    s = np.concatenate([[0], np.cumsum(d)])
    if s[-1] < 1e-9:
        return pts
    t = np.arange(0, s[-1], step)
    return np.stack([np.interp(t, s, pts[:, 0]), np.interp(t, s, pts[:, 1])], 1)


def metrics(m, path):
    pts = np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
    L = float(np.sum(np.hypot(*np.diff(pts, axis=0).T)))
    fine = resample(pts, 0.025)
    clr = np.array([m.clearance(x, y) for x, y in fine])
    unk = sum(m.is_unknown(x, y) for x, y in fine)
    r = resample(pts, 0.10)
    h = np.arctan2(np.diff(r[:, 1]), np.diff(r[:, 0]))
    th = np.abs((np.diff(h) + np.pi) % (2 * np.pi) - np.pi)
    ds = 0.10
    return dict(length=L, turning=float(th.sum()), bending=float(np.sum(th ** 2 / ds)),
                max_curv=float(th.max() / ds) if len(th) else 0.0,
                min_clear=float(clr.min()), mean_clear=float(clr.mean()),
                collision=bool(clr.min() < ROBOT_R), chassis_viol=bool(clr.min() < CHASSIS_R - 1e-3),
                unknown=int(unk))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--pairs', type=int, default=100)
    ap.add_argument('--seed', type=int, default=3)
    ap.add_argument('--min-dist', type=float, default=3.0)
    ap.add_argument('--planners', default='EMRA,EMRASeed,NavFn,Smac2D,ThetaStar')
    ap.add_argument('--csv', default='planner_benchmark.csv')
    a = ap.parse_args()
    planners = a.planners.split(',')

    m = Map(a.map)
    # sample reachable, comfortably safe points (largest connected component of the 0.2 m-safe region)
    safe = m.edt_blk > 0.30
    lab, n = ndimage.label(safe)
    big = np.argmax(np.bincount(lab.ravel())[1:]) + 1
    ys, xs = np.nonzero(lab == big)
    rng = np.random.default_rng(a.seed)
    pairs = []
    while len(pairs) < a.pairs:
        i, j = rng.integers(len(xs), size=2)
        s = (m.ox + (xs[i] + .5) * m.res, m.oy + (ys[i] + .5) * m.res)
        g = (m.ox + (xs[j] + .5) * m.res, m.oy + (ys[j] + .5) * m.res)
        if math.hypot(s[0] - g[0], s[1] - g[1]) >= a.min_dist:
            pairs.append((s, g))

    rclpy.init()
    node = Node('planner_benchmark')
    cli = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    if not cli.wait_for_server(timeout_sec=30):
        print('planner_server not available'); sys.exit(1)
    time.sleep(2.0)   # let the global costmap fill

    def pose(p):
        ps = PoseStamped(); ps.header.frame_id = 'map'
        ps.pose.position.x, ps.pose.position.y = p; ps.pose.orientation.w = 1.0
        return ps

    rows = []
    for k, (s, g) in enumerate(pairs):
        for pl in planners:
            goal = ComputePathToPose.Goal()
            goal.start = pose(s); goal.goal = pose(g); goal.use_start = True; goal.planner_id = pl
            t0 = time.time()
            fut = cli.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, fut)
            gh = fut.result()
            row = dict(pair=k, planner=pl, ok=False)
            if gh.accepted:
                rf = gh.get_result_async()
                rclpy.spin_until_future_complete(node, rf)
                res = rf.result().result
                wall = time.time() - t0
                if len(res.path.poses) > 1:
                    row.update(ok=True, time=wall, **metrics(m, res.path))
            rows.append(row)
        if (k + 1) % 10 == 0:
            print(f'{k + 1}/{len(pairs)} pairs', flush=True)

    keys = ['pair', 'planner', 'ok', 'time', 'length', 'turning', 'bending', 'max_curv', 'min_clear',
            'mean_clear', 'collision', 'chassis_viol', 'unknown']
    with open(a.csv, 'w', newline='') as f:
        w = csv.DictWriter(f, keys, extrasaction='ignore'); w.writeheader(); w.writerows(rows)

    # summary over pairs that EVERY planner solved (fair comparison)
    byp = {}
    for r in rows:
        byp.setdefault(r['pair'], {})[r['planner']] = r
    common = [k for k, d in byp.items() if all(d[p]['ok'] for p in planners)]
    print(f"\nmap={a.map}  pairs={len(pairs)}  solved-by-all={len(common)}")
    hdr = f"{'planner':10} {'success':>8} {'length m':>9} {'turning rad':>11} {'bending':>8} {'max curv':>8} " \
          f"{'min clr':>8} {'mean clr':>8} {'<0.11m':>7} {'<0.16m':>7} {'time s':>7}"
    print(hdr); print('-' * len(hdr))
    for pl in planners:
        allr = [r for r in rows if r['planner'] == pl]
        ok = sum(r['ok'] for r in allr)
        c = [byp[k][pl] for k in common]
        if not c:
            print(f"{pl:10} {ok}/{len(allr)}"); continue
        mean = lambda key: np.mean([r[key] for r in c])
        print(f"{pl:10} {ok:>3}/{len(allr):<4} {mean('length'):9.2f} {mean('turning'):11.2f} {mean('bending'):8.2f} "
              f"{mean('max_curv'):8.2f} {min(r['min_clear'] for r in c):8.3f} {mean('mean_clear'):8.3f} "
              f"{sum(r['collision'] for r in c):>7} {sum(r['chassis_viol'] for r in c):>7} {mean('time'):7.3f}")
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
