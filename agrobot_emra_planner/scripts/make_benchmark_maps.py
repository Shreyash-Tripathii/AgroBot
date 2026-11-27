#!/usr/bin/env python3
"""Generate synthetic benchmark maps (PGM+YAML): a crop-row field and a cluttered yard."""
import os
import sys

import numpy as np
from PIL import Image

RES = 0.05


def write(name, grid, out_dir, origin):
    img = np.full(grid.shape, 254, np.uint8)
    img[grid == 1] = 0
    Image.fromarray(np.flipud(img)).save(os.path.join(out_dir, name + '.pgm'))
    with open(os.path.join(out_dir, name + '.yaml'), 'w') as f:
        f.write(f"image: {name}.pgm\nmode: trinary\nresolution: {RES}\n"
                f"origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.25\n")


def rect(g, x0, y0, x1, y1):
    g[int(y0 / RES):int(y1 / RES) + 1, int(x0 / RES):int(x1 / RES) + 1] = 1


def main(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    # --- crop-row field 30 x 20 m: boundary wall, 8 rows (0.3 m wide, 1.0 m aisle), two headland gaps
    W, H = 30.0, 20.0
    g = np.zeros((int(H / RES), int(W / RES)), np.uint8)
    rect(g, 0, 0, W, 0.1); rect(g, 0, H - 0.1, W, H); rect(g, 0, 0, 0.1, H); rect(g, W - 0.1, 0, W, H)
    for k in range(8):
        y = 2.5 + k * 1.9
        rect(g, 3.0, y, 27.0, y + 0.3)
    write('bench_rows', g, out_dir, (0.0, 0.0))
    # --- cluttered yard 20 x 20 m, random boxes / pillars (fixed seed)
    rng = np.random.default_rng(7)
    W = H = 20.0
    g = np.zeros((int(H / RES), int(W / RES)), np.uint8)
    rect(g, 0, 0, W, 0.1); rect(g, 0, H - 0.1, W, H); rect(g, 0, 0, 0.1, H); rect(g, W - 0.1, 0, W, H)
    for _ in range(55):
        x, y = rng.uniform(1, 18), rng.uniform(1, 18)
        w, h = rng.uniform(0.2, 1.8), rng.uniform(0.2, 1.8)
        rect(g, x, y, x + w, y + h)
    write('bench_yard', g, out_dir, (0.0, 0.0))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '.')
