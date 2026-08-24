"""Post-mission evaluation: does communication-as-sensing actually help?

Reads the per-robot CSV logs written by the localizer and reports absolute
trajectory error (ATE, RMSE and max against ground truth) for the three
estimate tracks that ran simultaneously during the mission:

  A  pure dead reckoning (noisy odometry only)
  B  compass-aided dead reckoning        <- the no-RF baseline
  C  B + RSSI range factors from the mesh (peers + pad tags)  <- the system

The claim is falsifiable by construction: if C does not beat B, the RF
factors did nothing. Usage:

  ros2 run meshbots eval_metrics [--dir /tmp/meshbots_eval] [--plot out.png]
"""
import argparse
import csv
import glob
import math
import os


def load(path):
    rows = []
    with open(path) as f:
        r = csv.reader(f)
        header = next(r, None)
        for line in r:
            if len(line) >= 9:
                try:
                    rows.append([float(v) for v in line])
                except ValueError:
                    pass
    return rows


def ate(rows, ix, iy):
    errs = [math.hypot(r[ix] - r[1], r[iy] - r[2]) for r in rows]
    if not errs:
        return float('nan'), float('nan')
    rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
    return rmse, max(errs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default='/tmp/meshbots_eval')
    ap.add_argument('--plot', default='')
    args, _ = ap.parse_known_args()

    files = sorted(glob.glob(os.path.join(args.dir, 'rover_*.csv')))
    if not files:
        print(f'no logs found in {args.dir} — run a mission first')
        return

    tracks = [('A  pure dead reckoning', 3, 4),
              ('B  compass DR (no RF)  ', 5, 6),
              ('C  + RF range factors  ', 7, 8)]
    print(f'{"robot":<10}  {"track":<26}  {"ATE RMSE":>9}  {"max err":>9}')
    print('-' * 62)
    totals = {name: [] for name, _, _ in tracks}
    data = {}
    for path in files:
        robot = os.path.splitext(os.path.basename(path))[0]
        rows = load(path)
        data[robot] = rows
        for name, ix, iy in tracks:
            rmse, mx = ate(rows, ix, iy)
            totals[name].append(rmse)
            print(f'{robot:<10}  {name:<26}  {rmse:8.2f}m  {mx:8.2f}m')
        print('-' * 62)
    for name, _, _ in tracks:
        vals = [v for v in totals[name] if not math.isnan(v)]
        if vals:
            print(f'{"TEAM MEAN":<10}  {name:<26}  '
                  f'{sum(vals) / len(vals):8.2f}m')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(13, 6))
        colors = {'rover_1': 'tab:red', 'rover_2': 'tab:blue',
                  'rover_3': 'tab:orange'}
        for robot, rows in data.items():
            c = colors.get(robot, 'gray')
            axes[0].plot([r[1] for r in rows], [r[2] for r in rows],
                         color=c, lw=2, label=f'{robot} truth')
            axes[0].plot([r[3] for r in rows], [r[4] for r in rows],
                         color=c, lw=1, ls=':', alpha=0.8)
            axes[0].plot([r[7] for r in rows], [r[8] for r in rows],
                         color=c, lw=1, ls='--', alpha=0.9)
            t0 = rows[0][0] if rows else 0.0
            ts = [r[0] - t0 for r in rows]
            axes[1].plot(ts, [math.hypot(r[5] - r[1], r[6] - r[2])
                              for r in rows], color=c, ls=':',
                         label=f'{robot} no RF')
            axes[1].plot(ts, [math.hypot(r[7] - r[1], r[8] - r[2])
                              for r in rows], color=c, ls='-',
                         label=f'{robot} fused')
        axes[0].set_title('Trajectories (solid=truth, dotted=DR, dashed=fused)')
        axes[0].set_aspect('equal')
        axes[0].legend(fontsize=7)
        axes[1].set_title('Position error over time')
        axes[1].set_xlabel('t (s)')
        axes[1].set_ylabel('error (m)')
        axes[1].legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=130)
        print(f'plot written to {args.plot}')


if __name__ == '__main__':
    main()
