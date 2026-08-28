# meshbots

**Opportunistic communication-as-sensing for infrastructure-free multi-robot
teams — a reproducible ROS 2 / Gazebo research testbed.**

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Status: active research](https://img.shields.io/badge/status-active_research-purple)](docs/RESULTS.md)

A team of robots working where there is no infrastructure (disaster zones,
farms, other planets) must route its own communications: every robot is a
node in an ad-hoc mesh. Robotics treats that mesh as plumbing. This project
treats it as a **sensor the team already carries**: every relayed packet
arrives with a received signal strength (RSSI) that constrains the distance
between two robots, whose excess attenuation reveals what stands between
them, and which — observed through the robot's own motion — **calibrates the
wheel odometry the robot never had a way to check.** No extra hardware; the
measurements ride on the map updates, beacons and task auctions the team was
sending anyway — and every relayed packet is a rover acting as
infrastructure.

<p align="center">
  <img src="docs/bias_calibration.png" width="100%" alt="Left: wheel-scale bias estimated from packet RSSI (solid) converges toward the true injected bias (dashed) within ~50 s, 24 robot-runs. Right: team position error over time, self-calibrating fusion (green) vs position-only fusion (gray).">
  <br><sub><b>Fig. 1 —</b> Left: each rover's hidden 4–9 % wheel-scale bias (dashed) and its estimate from packet RSSI alone (solid), 24 robot-runs. Right: team position error, position-only RF fusion (gray) vs. self-calibrating fusion (green), one line per mission.</sub>
</p>

---

## Contents

1. [Key results](#key-results)
2. [What the system does](#what-the-system-does)
3. [Research questions and contributions](#research-questions-and-contributions)
4. [Installation](#installation)
5. [Running a mission](#running-a-mission)
6. [Reproducing the results](#reproducing-the-results)
7. [Evaluation protocol](#evaluation-protocol)
8. [Repository structure](#repository-structure)
9. [Architecture](#architecture)
10. [Configuration](#configuration)
11. [Status and roadmap](#status-and-roadmap)
12. [Limitations](#limitations)
13. [Documentation](#documentation)
14. [Citation, license, contact](#citation-license-contact)

---

## Key results

All numbers: 8 seeded missions per arm, 280 s window, three rovers, three
delivery pads, no central coordinator. Four localization estimators run
**simultaneously in every mission on identical degraded odometry and
identical packets**, so every run is its own paired ablation.
**Team ATE** = RMSE of each robot's position estimate against Gazebo ground
truth over the whole 280 s window, averaged over the three robots.
Details, per-seed values and caveats in [`docs/RESULTS.md`](docs/RESULTS.md);
raw logs of every run in [`results/`](results/).

**Campaign 3 — the mesh calibrates the wheels** (fixed wedge, 8/8 missions
delivered 3/3):

| track | estimator | uses the mesh? | team ATE (mean ± std) |
|---|---|---|---|
| A | pure dead reckoning | no | 1.99 ± 0.29 m |
| B | compass-aided dead reckoning | no — **baseline** | 0.97 ± 0.12 m |
| C | B + RSSI range factors (peers + pad tags) | yes | 0.57 ± 0.12 m |
| **D** | C + wheel-scale bias as an EKF state | yes | **0.46 ± 0.12 m** |

| paired comparison (same seed, same packets) | ATE reduction | better in |
|---|---|---|
| C vs B — range factors harvested from mission traffic | 41 % ± 13 % | 8/8 |
| **D vs C — adding the bias state** | **20 % ± 8 %** | **8/8** |
| D vs B — the whole communication-as-sensing layer | 52 % ± 15 % | 8/8 |

- **Self-calibration:** the injected wheel-scale bias (mean magnitude 6.5 %,
  random sign) is recovered to a residual of **1.1 % ± 0.5 %** over 24
  robot-runs, converging within ~50 s of motion — from packet RSSI only.
- **Mapping:** 95 % arena coverage with mesh-merged patches vs. 87 % from a
  robot's own lidar. RF-shadow evidence marks obstacles no lidar has seen.
- **Mesh:** ≈ 4,200 packets originated, ≈ 8,400 relayed hop-by-hop per
  mission.
- **Why C-vs-B reads 41 % here and 34 % in Campaign 1:** the two campaigns
  are not the same estimator. Campaign 1 predates the consistency
  safeguards (peer-noise inflation, covariance floor) and the seeded
  channel, and its robots were driven by track C; in Campaign 3 the robots
  are driven by track D, so they follow different trajectories. Each figure
  is paired within its own campaign; they are not comparable across
  campaigns.

**Campaign 2 — formation as aperture** (paired A/B, 8 + 8 missions):
letting followers perturb their formation slots to make their links
informative harvested measurably *more* RF information (median paired
correction 44 % vs. 36 %) but the extra manoeuvring integrated as much
odometry drift as it removed and cost mission time. **Reported as a
negative result together with its mechanism:** an information-gain planner
amplifies any inconsistency in the estimator it feeds, and our consistency
safeguard (a covariance floor) had silently removed the geometric signal
the planner depended on.

> **The caveat that matters.** These numbers come from an idealized channel
> (log-distance path loss, 2 dB i.i.d. shadowing). Real indoor WiFi shows
> 3–6 dB with multipath and per-device offsets. Campaign 4 sweeps 2/4/6/8 dB
> shadowing, temporally correlated fading and unknown per-device offsets,
> and will be reported here whether or not the gains survive. Absolute
> errors will not transfer to hardware; only the paired deltas are meant to.

## What the system does

Three rovers execute a delivery mission (three pads, return to base) while:

| capability | how |
|---|---|
| **Mesh networking** | Every rover is a MANET node: packets flood hop-by-hop with TTL and duplicate suppression. Two rovers out of range talk *through* the third. No base station. |
| **Cooperative localization from traffic** | Odometry is deliberately degraded (seeded per-robot scale bias + noise). Each rover corrects itself with range factors inverted from the RSSI of *direct* packets: from teammates (whose broadcast poses carry covariance) and from transmit-only RF tags on the delivery pads. Links whose ray crosses a known obstacle are gated out — *lidar informing RF*. |
| **Odometry self-calibration** | The wheel velocity-scale bias is an EKF state. Range factors observe it through the motion-model Jacobian, so the mesh calibrates the encoders. |
| **Collaborative mapping** | Each rover raytraces lidar into a log-odds grid and shares only changed cells, **exclusively over the mesh**; peers merge them (own observation beats hearsay). Partition the mesh and the maps stop converging. |
| **RF-shadow mapping** | A link that loses more signal than free space predicts means something stands between the robots. The excess votes obstruction along the ray into cells no lidar has seen — *RF informing the map*. |
| **Squadron behaviour** | Heartbeat leader election with failover (kill the leader; the next rover takes command and finishes the mission), wedge formation from the leader's *mesh-beaconed* estimate, sealed-bid delivery auctions with load balancing, a mission FSM replicated on every robot. |
| **Docking by radio** | The pad's own chirp doubles as the docking sensor: RSSI above a contact threshold means the robot is physically on the pad, whatever its estimate believes. |
| **Formation-as-aperture planner** (optional arm) | Followers score perturbations of their slot by predicted link information — range-factor variance reduction plus unknown map cells the link would sweep — minus deviation and motion cost, under a leader-link PDR floor. |

Every per-robot node runs identical code under its own namespace. The only
global process is the RF propagation simulator, which models physics — path
loss, obstacle penetration, shadowing, fading, device offsets, packet loss,
per-packet RSSI stamping — and contains no coordination logic. Robots never
see ground truth.

## Research questions and contributions

1. **How much localization, mapping and calibration value can a team recover
   for free from packets it was going to send anyway?** → Campaigns 1 & 3.
2. **Does it pay to move, within the mission's tolerance, to make those
   packets more informative?** ("Formation as aperture") → Campaign 2, and
   a cost-aware re-run on the improved estimator (Campaign 3, arms 2–3).
3. **Where do the gains die as the channel gets realistic?** → Campaign 4.

Claimed contributions (see [`docs/RELATED.md`](docs/RELATED.md) for the
literature and an honest novelty statement — the ingredients are not new;
the combination, the calibration result and the findings are):

- A fully decentralized system in which opportunistic per-packet RSSI from
  mission traffic serves at once as a localization factor, a mapping
  modality and an odometry-calibration signal, with lidar↔RF gating in
  both directions.
- A paired Monte Carlo protocol in which every mission is its own ablation
  (four estimator tracks on identical measurements) and every formation arm
  is seed-paired; all raw data committed.
- An online formation-slot perturbation planner with a link-ray coverage
  term, its paired A/B result, and a system-level account of how active
  information-gain planning interacts with estimator consistency.

## Installation

Tested on Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (gz-sim 8), Python 3.12.

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz python3-numpy python3-matplotlib
git clone https://github.com/MazharZiadeh/meshbots.git
cd meshbots/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Optional — Husarion ROSbot chassis (scaffolded, untested):
`./scripts/get_rosbot_sources.sh` fetches the vendored `rosbot_ros` and
`husarion_gz_worlds` sources (git-ignored); launch with `chassis:=rosbot`.

## Running a mission

```bash
ros2 launch meshbots sim.launch.py                          # Gazebo GUI + RViz
ros2 launch meshbots sim.launch.py gui:=false               # headless sim, RViz on
ros2 launch meshbots sim.launch.py gui:=false rviz:=false   # fully headless
```

The mission runs itself: the squadron forms up, sweeps to each red pad,
auctions the delivery, the winner docks, the pad turns green, the team
returns to base. Typical completion ≈ 2 minutes; evaluation window 280 s.

| launch argument | values | meaning |
|---|---|---|
| `seed` | integer | reproducible odometry-degradation and channel draws (same `(robot, seed)` → same noise character) |
| `formation` | `fixed` · `informative` · `aware` | wedge only · aperture planner · cost-aware aperture planner |
| `rf_sigma_db` | dB (default 2.0) | i.i.d. shadowing std of the channel |
| `rf_fading_db`, `rf_fading_tau` | dB, s (default 0, 5) | temporally correlated (AR(1)) slow fading per link |
| `rf_offset_db` | dB (default 0) | per-device TX/RX offsets, uniform ±, unknown to the robots |
| `chassis` | `builtin` · `rosbot` | simple diff-drive rover · Husarion ROSbot stack |
| `eval_dir` | path | where per-run CSV logs are written |

Things worth trying during a run:

```bash
ros2 topic echo /rover_1/mesh/rx | grep -m5 path      # multi-hop routes actually taken
ros2 topic echo /rover_2/mesh/stats                    # tx / rx / relayed / dups
pkill -f "swarm.*__ns:=/rover_1"                       # kill the leader: watch failover
pkill -f "mesh_radio.*__ns:=/rover_2"                  # partition the mesh: rover_2's
                                                       #   merged_map stops growing from peers
ros2 run meshbots eval_metrics --plot ate.png          # ATE of the run so far
```

In RViz: green↔red lines are live mesh links coloured by predicted delivery
ratio (watch one redden as a silo slides between rovers); mid-gray cells in
`merged_map` are RF-suspected obstacles no lidar has confirmed; labels show
each rover's role and phase.

## Reproducing the results

Every table in this README and in `docs/RESULTS.md` was produced by these
commands, and the raw per-run CSVs behind them are committed under
`results/<campaign>/run_<seed>/`.

```bash
# One campaign: N seeded missions, one directory each (~5 min per mission)
./scripts/run_batch.sh 8 results/my_fixed 280
./scripts/run_batch.sh 8 results/my_aware 280 formation:=aware
./scripts/run_batch.sh 8 results/my_harsh 280 rf_sigma_db:=6.0

# Statistics
ros2 run meshbots batch_metrics --dir results/my_fixed --markdown --plot mc.png
ros2 run meshbots batch_metrics --compare results/my_fixed results/my_aware --markdown
python3 scripts/plot_bias.py results/my_fixed bias.png

# The exact campaign scripts behind the reported numbers
./scripts/run_campaign3.sh        # fixed / aware / informative, 8 seeds each
./scripts/run_sensitivity.sh      # channel-harshness sweep (Campaign 4)
```

Per run you get `rover_N.csv` (ground truth and all four estimator tracks at
2 Hz, plus estimated and true wheel bias), `rover_N_map.csv` (own vs merged
coverage), `rover_N_mesh.csv` (traffic counters), `mission_*.csv`
(completion time), `deliveries.txt` and `launch.log`.

## Evaluation protocol

- **Paired by construction.** The four localization tracks (A pure DR, B
  compass DR, C RF fusion, D RF fusion + bias state) run inside the same
  mission on the same degraded odometry and consume the same accepted range
  factors; the RF-vs-no-RF and bias-vs-no-bias comparisons are therefore
  paired measurement by measurement. Formation arms are paired by seed.
- **Metric.** Absolute trajectory error (RMSE against Gazebo ground truth)
  per robot over the full window including the parked tail, averaged over
  the team; map coverage at end of window; mission completion time and
  deliveries within the window; mesh traffic counters.
- **Reproducibility.** Per-`(robot, seed)` noise draws via CRC32-seeded
  RNGs; the channel simulator is seeded from the same `seed`; the
  simulation runs at real-time factor 1 with wall-clock windows.
- **Falsifiability.** If D does not beat C, or C does not beat B, the
  corresponding claim is false and the tables will say so. Campaign 2 is an
  example of the protocol returning a negative answer.

## Repository structure

```
ros2_ws/src/meshbots/
├── meshbots/
│   ├── mesh_radio.py         # per-robot MANET radio: flood, relay, TTL, dedup, hop paths
│   ├── radio_channel.py      # the ONLY global node — RF physics ("the air"), never coordination
│   ├── rf_model.py           # shared path-loss / penetration model, mirrors worlds/arena.sdf
│   ├── localizer.py          # four-track cooperative localizer (RangeEKF: [x,y] and [x,y,bias])
│   ├── mapper.py             # collaborative log-odds grid, mesh patch exchange, RF-shadow layer
│   ├── swarm.py              # beacons, leader election, wedge formation, auctions, mission FSM
│   ├── formation_planner.py  # formation-as-aperture slot perturbation (optional arm)
│   ├── navigator.py          # potential-field local planner
│   ├── eval_metrics.py       # single-run ATE
│   └── batch_metrics.py      # campaign statistics, paired comparisons, plots
├── launch/                   # sim.launch.py (top level) · gazebo.launch.py · rviz.launch.py
├── config/                   # team · swarm · localization · mapping · navigation · mesh · formation · formation_aware
├── worlds/arena.sdf · models/rover.sdf.template · missions/delivery.yaml · rviz/
scripts/                      # run_batch.sh · run_campaign3.sh · run_sensitivity.sh · plot_bias.py · get_rosbot_sources.sh
docs/                         # RESULTS.md · PAPER.md · IDEA.md · RELATED.md · figures
results/                      # raw per-run logs of every reported campaign
```

Structured after the Husarion ROS 2 tutorial conventions: one package,
small composable launch files, every tunable in `config/*.yaml`, nothing
hard-coded. Adding a fourth rover is one line in `config/team.yaml` plus a
formation slot in `config/swarm.yaml`.

## Architecture

```
                 ┌────────────────────────── per rover ×3 ──────────────────────────┐
                 │                                                                  │
  Gazebo         │  scan ──► mapper ◄── /pose ── localizer ◄── odom (degraded DR)   │
  (arena,        │            │  ▲ map patches      ▲  RSSI range factors           │
  diff-drive,    │            │  └──────┐           │  + bias state, RF-shadow rays │
  gpu lidar)     │            ▼         │           │                               │
                 │       merged_map   mesh/rx ◄── mesh_radio ◄─┐ (flood relay,      │
                 │            ▲          ▲            ▲        │  TTL, dedup)       │
                 │  formation_planner ─► swarm ── beacons ─────┘                    │
                 │   (slot_offset)        │ election · formation · auction          │
                 │                        ▼                                         │
                 │  goal ──► navigator ──► cmd_vel                                  │
                 └──────────────────────────────────────────────┼───────────────────┘
                                                                │ air_tx / air_rx
                                               radio_channel  ◄─┘
                              (RF physics only: log-distance path loss, obstacle
                               penetration, shadowing / fading / device offsets,
                               PDR, per-packet RSSI stamping, pad-tag chirps)
```

Estimation in one paragraph: for a direct packet with RSSI *r* the
log-distance model gives d̂ = 10^((P_tx − PL₀ − r)/10n) with
σ_d ≈ d·ln10·σ_dB/(10n). The localizer runs a scalar-EKF range update on
[x, y] (track C) and on [x, y, b] (track D), where the prediction uses
v_m/(1 + b) so the Jacobian ∂x/∂b builds position–bias correlation along
the direction of travel and every range factor also corrects b. Peer
factors get inflated noise and a covariance floor as a cheap stand-in for
covariance intersection. Rays through known obstacles are gated; rays with
excess loss above 3σ_dB vote obstruction into the map.

## Configuration

| file | what lives there |
|---|---|
| `config/team.yaml` | who exists and where they spawn |
| `config/swarm.yaml` | formation slots, auction timing, load penalty, docking RSSI |
| `config/localization.yaml` | odometry degradation ranges, EKF tuning, consistency safeguards, bias-state prior, `drive_track` (C or D), `rf_factors` ablation switch |
| `config/mapping.yaml` | grid geometry, patch caps, RF-shadow vote threshold |
| `config/formation.yaml`, `config/formation_aware.yaml` | aperture-planner weights, trust radius, PDR floor, motion cost, uncertainty gate, hold time |
| `config/mesh.yaml`, `config/navigation.yaml` | radio TTL; planner gains |
| `missions/delivery.yaml` | the op order: targets and base |

## Status and roadmap

| campaign | question | status |
|---|---|---|
| 1 | RF range factors vs no-RF (n = 8) | done — 34 % ± 19 %, 8/8 |
| 2 | formation-as-aperture paired A/B (8 + 8) | done — negative result, mechanism identified |
| 3 | odometry self-calibration; cost-aware planner on the improved estimator | fixed arm done (above); planner arms running |
| 4 | channel-harshness sweep (2/4/6/8 dB, fading, offsets) | queued, script ready |
| — | covariance intersection for peer factors (replace inflation + floor) | queued |
| — | per-device RSSI offset as a calibrated state | planned |
| — | n = 16 headline arm; LaTeX preprint (arXiv cs.RO) | planned |
| — | hardware RSSI characterization (ESP32) | wanted |

## Limitations

- **Simulation only, idealized RF at the headline point.** Log-distance +
  penetration with 2 dB i.i.d. shadowing; no multipath or antenna
  patterns. Campaign 4 characterizes degradation; hardware is the necessary
  next step before any strong claim transfers.
- **Channel parameters assumed known** (P_tx, PL₀, n). Field deployment
  needs online range-model calibration.
- **Correlated estimates.** Peer range updates ignore cross-correlation
  (the classic cooperative-localization consistency problem). Inflation
  and a covariance floor bound it but pin the covariance — which is what
  starved the formation planner. Covariance intersection is the fix.
- **Shared map origin** (surveyed base); real C-SLAM earns the transform
  through inter-robot loop closures.
- **Greedy planning**, one-step lookahead, no joint optimization across
  followers; reactive potential-field navigation.
- **Scale.** n = 8 per arm, one arena, three robots; statistics are
  indicative, not confidence intervals. The harness makes larger n one
  argument.

## Documentation

- [`docs/RESULTS.md`](docs/RESULTS.md) — every campaign, every table, caveats.
- [`docs/PAPER.md`](docs/PAPER.md) — the working paper draft.
- [`docs/IDEA.md`](docs/IDEA.md) — the idea in one page.
- [`docs/RELATED.md`](docs/RELATED.md) — literature pass, what is and is
  not new, reviewer risks, numbers to reuse.

## Citation, license, contact

If you use the testbed or the protocol, please cite the repository:

```bibtex
@misc{ziadeh2026meshbots,
  author = {Ziadeh, Mazhar},
  title  = {meshbots: Opportunistic Communication-as-Sensing for
            Infrastructure-Free Multi-Robot Teams},
  year   = {2026},
  howpublished = {\url{https://github.com/MazharZiadeh/meshbots}},
  note   = {Open-source ROS 2 / Gazebo research testbed; working paper in docs/}
}
```

MIT license. Issues, questions and collaboration are welcome through
GitHub issues.
