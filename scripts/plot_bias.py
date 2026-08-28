#!/usr/bin/env python3
"""Figure: the mesh calibrates the wheels.

Left: estimated odometry scale bias vs. time for every robot in every run
of a campaign (thin lines), against the true injected bias (dashed, same
colour). Right: position error vs. time, team mean per run, for the
position-only fusion (C) and the self-calibrating fusion (D).

  python3 scripts/plot_bias.py results/c3_fixed docs/bias_calibration.png
"""
import csv
import glob
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def load(path):
    with open(path) as f:
        r = csv.reader(f)
        next(r, None)
        rows = []
        for line in r:
            try:
                rows.append([float(v) for v in line])
            except ValueError:
                pass
    return rows


def main(base, out):
    runs = sorted(glob.glob(os.path.join(base, 'run_*')),
                  key=lambda p: int(p.rsplit('_', 1)[-1]))
    fig, (ax_b, ax_e) = plt.subplots(1, 2, figsize=(12.5, 4.8))
    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
    n_rob = 0
    for run in runs:
        files = [p for p in sorted(glob.glob(os.path.join(run, 'rover_*.csv')))
                 if '_map' not in p and '_mesh' not in p]
        err_c, err_d = {}, {}
        for i, path in enumerate(files):
            rows = load(path)
            if not rows or len(rows[0]) < 14:
                continue
            t0 = rows[0][0]
            ts = [r[0] - t0 for r in rows]
            col = colors[i % len(colors)]
            ax_b.plot(ts, [100 * r[12] for r in rows], color=col, alpha=0.5, lw=1)
            ax_b.plot([ts[0], ts[-1]], [100 * rows[0][13]] * 2, color=col,
                      ls='--', alpha=0.5, lw=1)
            n_rob += 1
            for r, t in zip(rows, ts):
                k = int(t // 5)
                err_c.setdefault(k, []).append(math.hypot(r[7] - r[1], r[8] - r[2]))
                err_d.setdefault(k, []).append(math.hypot(r[10] - r[1], r[11] - r[2]))
        ks = sorted(err_c)
        ax_e.plot([5 * k for k in ks], [sum(err_c[k]) / len(err_c[k]) for k in ks],
                  color='tab:gray', alpha=0.45, lw=1)
        ax_e.plot([5 * k for k in ks], [sum(err_d[k]) / len(err_d[k]) for k in ks],
                  color='tab:green', alpha=0.6, lw=1)
    ax_b.set_xlabel('mission time (s)')
    ax_b.set_ylabel('wheel-odometry scale bias (%)')
    ax_b.set_title(f'Estimated (solid) vs. true (dashed) bias, '
                   f'{n_rob} robot-runs')
    ax_b.grid(alpha=0.3)
    ax_e.plot([], [], color='tab:gray', label='C: RF factors, position only')
    ax_e.plot([], [], color='tab:green', label='D: RF factors + bias state')
    ax_e.set_xlabel('mission time (s)')
    ax_e.set_ylabel('team mean position error (m)')
    ax_e.set_title('Position error, one line per run')
    ax_e.legend()
    ax_e.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print('wrote', out, 'from', len(runs), 'runs')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else 'bias_calibration.png')
