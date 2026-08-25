"""Aggregate Monte Carlo mission runs into research-grade statistics.

Reads the per-run directories produced by scripts/run_batch.sh and reports,
across all runs: mission success, ATE mean +/- std per localization track,
the paired per-run improvement of RF factors over the no-RF baseline,
merged-vs-own map coverage, and mesh traffic totals.

  ros2 run meshbots batch_metrics [--dir /tmp/meshbots_batch] [--plot out.png]
  ros2 run meshbots batch_metrics --markdown > docs/RESULTS.md
"""
import argparse
import csv
import glob
import math
import os
import statistics as st

from .eval_metrics import load, ate

TRACKS = [('A pure dead reckoning', 3, 4),
          ('B compass DR (no RF)', 5, 6),
          ('C + RF range factors', 7, 8)]


def read_run(run_dir):
    out = {'name': os.path.basename(run_dir)}
    try:
        out['delivered'] = int(open(os.path.join(run_dir, 'deliveries.txt')).read())
    except (OSError, ValueError):
        out['delivered'] = None
    # Team ATE per track (mean of per-robot RMSE).
    per_track = {name: [] for name, _, _ in TRACKS}
    for path in sorted(glob.glob(os.path.join(run_dir, 'rover_*.csv'))):
        if '_map' in path or '_mesh' in path:
            continue
        rows = load(path)
        for name, ix, iy in TRACKS:
            rmse, _ = ate(rows, ix, iy)
            if not math.isnan(rmse):
                per_track[name].append(rmse)
    out['ate'] = {n: (st.mean(v) if v else float('nan'))
                  for n, v in per_track.items()}
    # Map coverage: final own vs merged known fraction (mean over robots).
    own, merged, curves = [], [], []
    for path in sorted(glob.glob(os.path.join(run_dir, 'rover_*_map.csv'))):
        with open(path) as f:
            rows = [r for r in csv.reader(f)][1:]
        if rows:
            t0 = float(rows[0][0])
            total = float(rows[-1][3])
            own.append(float(rows[-1][1]) / total)
            merged.append(float(rows[-1][2]) / total)
            curves.append([(float(r[0]) - t0, float(r[2]) / total,
                            float(r[1]) / total) for r in rows])
    out['cov_own'] = st.mean(own) if own else float('nan')
    out['cov_merged'] = st.mean(merged) if merged else float('nan')
    out['cov_curves'] = curves
    # Mesh traffic: totals of the final cumulative counters.
    tx = rx = relayed = 0
    for path in sorted(glob.glob(os.path.join(run_dir, 'rover_*_mesh.csv'))):
        with open(path) as f:
            rows = [r for r in csv.reader(f)][1:]
        if rows:
            tx += int(rows[-1][1])
            rx += int(rows[-1][2])
            relayed += int(rows[-1][3])
    out['mesh'] = (tx, rx, relayed)
    return out


def mean_std(vals):
    vals = [v for v in vals if not math.isnan(v)]
    if not vals:
        return float('nan'), float('nan')
    return st.mean(vals), (st.stdev(vals) if len(vals) > 1 else 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default='/tmp/meshbots_batch')
    ap.add_argument('--plot', default='')
    ap.add_argument('--markdown', action='store_true')
    args, _ = ap.parse_known_args()

    run_dirs = sorted(glob.glob(os.path.join(args.dir, 'run_*')),
                      key=lambda p: int(p.rsplit('_', 1)[-1]))
    runs = [read_run(d) for d in run_dirs]
    runs = [r for r in runs if not math.isnan(r['ate'][TRACKS[0][0]])]
    if not runs:
        print(f'no usable runs found under {args.dir}')
        return
    n = len(runs)

    md = args.markdown
    sep = '|' if md else '  '

    print(f'# Monte Carlo results — {n} missions, seeds '
          f'{runs[0]["name"].split("_")[-1]}–{runs[-1]["name"].split("_")[-1]}'
          if md else f'=== {n} missions ===')
    ok = sum(1 for r in runs if r['delivered'] == 3)
    print()
    print(f'**Mission success:** {ok}/{n} runs delivered all 3 targets.'
          if md else f'mission success: {ok}/{n} delivered 3/3')
    print()

    if md:
        print('| track | team ATE mean | std | min | max |')
        print('|---|---|---|---|---|')
    for name, _, _ in TRACKS:
        vals = [r['ate'][name] for r in runs]
        m, s = mean_std(vals)
        lo, hi = min(vals), max(vals)
        if md:
            print(f'| {name} | {m:.2f} m | {s:.2f} | {lo:.2f} | {hi:.2f} |')
        else:
            print(f'{name:<24}{sep}{m:6.2f} m ±{s:.2f}  [{lo:.2f}, {hi:.2f}]')
    print()

    # Paired improvement, C vs B, per run (same seed -> fair pairing).
    imps = [100.0 * (r['ate'][TRACKS[1][0]] - r['ate'][TRACKS[2][0]])
            / r['ate'][TRACKS[1][0]] for r in runs
            if r['ate'][TRACKS[1][0]] > 0]
    m, s = mean_std(imps)
    wins = sum(1 for i in imps if i > 0)
    print((f'**RF factors vs no-RF baseline (paired per seed):** '
           f'{m:.0f}% ± {s:.0f}% ATE reduction; better in {wins}/{n} runs.')
          if md else
          f'paired C-vs-B improvement: {m:.0f}% ±{s:.0f}%  ({wins}/{n} runs better)')
    print()

    co, _ = mean_std([r['cov_own'] for r in runs])
    cm, _ = mean_std([r['cov_merged'] for r in runs])
    print((f'**Map coverage (end of mission, mean per robot):** '
           f'{100 * cm:.0f}% with mesh merging vs {100 * co:.0f}% from own '
           f'lidar alone (+{100 * (cm - co):.0f} points from teammates).')
          if md else
          f'coverage: merged {100 * cm:.0f}% vs own {100 * co:.0f}%')
    print()

    tx, _ = mean_std([r['mesh'][0] for r in runs])
    rx, _ = mean_std([r['mesh'][1] for r in runs])
    rl, _ = mean_std([r['mesh'][2] for r in runs])
    print((f'**Mesh traffic per mission (team totals):** {tx:.0f} packets '
           f'originated, {rx:.0f} received, {rl:.0f} relayed hop-by-hop.')
          if md else
          f'mesh per mission: tx {tx:.0f}, rx {rx:.0f}, relayed {rl:.0f}')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.2))
        # Left: ATE per track, one dot per run + mean bar.
        for k, (name, _, _) in enumerate(TRACKS):
            vals = [r['ate'][name] for r in runs]
            axes[0].scatter([k] * len(vals), vals, alpha=0.65, zorder=3)
            m, s = mean_std(vals)
            axes[0].errorbar([k], [m], yerr=[s], fmt='_', ms=28, lw=2,
                             color='black', capsize=6, zorder=4)
        axes[0].set_xticks(range(len(TRACKS)))
        axes[0].set_xticklabels([t[0].split(' ', 1)[0] for t in TRACKS])
        axes[0].set_ylabel('team ATE RMSE (m)')
        axes[0].set_title(f'Localization error, {n} seeded missions '
                          '(dots = runs, bar = mean ± std)')
        axes[0].grid(axis='y', alpha=0.3)
        # Right: coverage curves, merged vs own.
        for r in runs:
            for curve in r['cov_curves']:
                ts = [c[0] for c in curve]
                axes[1].plot(ts, [100 * c[1] for c in curve],
                             color='tab:green', alpha=0.35, lw=1)
                axes[1].plot(ts, [100 * c[2] for c in curve],
                             color='tab:gray', alpha=0.35, lw=1)
        axes[1].plot([], [], color='tab:green', label='merged (with mesh)')
        axes[1].plot([], [], color='tab:gray', label='own lidar only')
        axes[1].set_xlabel('mission time (s)')
        axes[1].set_ylabel('map coverage (%)')
        axes[1].set_title('Every robot map, every run')
        axes[1].legend()
        axes[1].grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=130)
        print(f'\nplot written to {args.plot}')


if __name__ == '__main__':
    main()
