# meshbots — Decentralized Mesh Multi-Robot Delivery

**Communication-as-sensing for a decentralized multi-robot team: mesh-relayed
comms, collaborative mapping, cooperative localization from link RSSI, and
formation control — with no central coordinator.**

Three rovers deliver payloads across an arena while:

- **meshing** — every rover is a network node. Packets flood hop-by-hop
  (TTL + duplicate suppression, MANET-style); two rovers out of radio range
  talk *through* the third. No base station, no central antenna.
- **mapping** — each rover raytraces its lidar into a local occupancy grid
  and shares only *changed cells* ("map patches") **exclusively over the
  mesh**. Every rover merges peer patches into its own fused world map.
  Partition the mesh and the maps visibly stop converging.
- **localizing cooperatively** — odometry is deliberately noisy; each rover
  corrects its estimate with **RSSI-derived range factors** from the very
  packets the mesh is already exchanging (ISAC: the link is also a sensor),
  plus chirps from cheap transmit-only RF tags on the delivery pads.
- **moving like a squadron** — heartbeat-based leader election (kill the
  leader; the next rover takes over and resumes the mission), wedge
  formation where followers steer to slots computed from the leader's
  *mesh-beaconed* pose estimate — localization independent, yet dependent.
- **allocating tasks by market** — at each pad, every rover broadcasts a bid
  (distance + load-balancing penalty); everyone computes the same argmin
  locally, so the team agrees on the deliverer without a master.
- **RF-shadow mapping** — when a link measures more loss than free space
  predicts, *something is standing between the robots*. Those excess-loss
  rays accumulate into "suspected obstacle" evidence — the mesh maps what
  lidar hasn't seen yet.

Stack: **ROS 2 Jazzy · Gazebo Harmonic (gz-sim 8) · Python / rclpy**.
Everything per-robot is identical code under its own namespace; the only
global process is the RF propagation simulator, which models physics
(path loss, penetration attenuation, packet loss), never coordination.

---

## Quick start

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch meshbots sim.launch.py                 # Gazebo GUI + RViz
ros2 launch meshbots sim.launch.py gui:=false      # headless sim + RViz
```

The mission runs itself: the squadron forms up, sweeps to each red pad,
auctions the delivery, the winner docks and delivers (pad marker turns
green), and the team returns to base. A full mission takes ~4 minutes.

**After (or during) a run — the numbers:**

```bash
ros2 run meshbots eval_metrics --plot /tmp/ate.png
```

prints absolute trajectory error for the three estimators that ran
simultaneously (the ablation is built into every mission):

| track | estimator | uses the mesh? |
|---|---|---|
| A | pure dead reckoning | no |
| B | compass-aided dead reckoning | no — the baseline |
| C | B + RSSI range factors (peers + pad tags) | **yes** |

If C doesn't beat B, communication-as-sensing did nothing — the claim is
falsifiable by construction.

## What to watch in RViz

- **Green↔red lines** between rovers: live mesh links, colored by predicted
  packet-delivery ratio from the path-loss model. Watch a link redden as a
  silo slides between two rovers.
- **The merged map** (`/rover_1/merged_map`): grows from three lidars at
  once — enable rover_2/rover_3's maps to compare each robot's world view.
  Mid-gray cells (value 60) are **RF-suspected obstacles** no lidar has
  confirmed. Raw RF evidence: `/rover_N/rf_map`.
- **Labels** over each rover: role (LEADER/follower) and mission phase.
- Red pads = pending deliveries; green = delivered.

## Things worth trying

```bash
# Kill the leader mid-mission — watch failover (rover_2 takes command):
ros2 topic pub -r 10 /rover_1/cmd_vel geometry_msgs/msg/Twist "{}" &   # or:
ros2 lifecycle set ...   # simplest: kill rover_1's swarm node process

# Watch multi-hop relaying (the "path" field shows actual routes):
ros2 topic echo /rover_1/mesh/rx | grep -m5 path

# Mesh traffic stats (tx/rx/relayed/dups):
ros2 topic echo /rover_2/mesh/stats
```

## Architecture

```
                 ┌────────────────────────── per rover ×3 ──────────────────────────┐
                 │                                                                  │
  Gazebo         │  scan ──► mapper ◄── /pose ── localizer ◄── odom (noisy DR)      │
  (arena,        │            │  ▲ map patches      ▲  RSSI range factors           │
  diff-drive,    │            │  └──────┐           │  + RF-shadow rays             │
  gpu lidar)     │            ▼         │           │                               │
                 │       merged_map   mesh/rx ◄── mesh_radio ◄─┐ (flood relay,      │
                 │                      ▲            ▲         │  TTL, dedup)       │
                 │  swarm ──────────────┴─ beacons ──┘         │                    │
                 │   │ leader election · formation · auction   │                    │
                 │   ▼                                         │                    │
                 │  goal ──► navigator ──► cmd_vel             │                    │
                 └─────────────────────────────────────────────┼────────────────────┘
                                                               │ air_tx / air_rx
                                              radio_channel  ◄─┘
                              (RF physics only: log-distance path loss,
                               obstacle penetration, PDR, RSSI stamping,
                               delivery-pad RF tag chirps)
```

- `mesh_radio.py` — per-robot MANET radio: wraps app payloads into packets,
  floods, relays with TTL, suppresses duplicates, records hop paths.
- `radio_channel.py` — "the air": for each (tx, potential rx) pair, samples
  RSSI = P_tx − PL(d) − Σ(wall attenuation) + N(0,σ) against the arena
  geometry and delivers with probability PDR(RSSI). Robots only ever see
  what their radio heard — never ground truth.
- `localizer.py` — noisy dead reckoning + scalar-EKF range updates from the
  RSSI of *direct* (single-hop) packets; links whose ray crosses a
  known-occupied map cell are gated out (lidar informing RF). Emits
  excess-loss rays for RF-shadow mapping (RF informing the map).
- `mapper.py` — log-odds occupancy grid, mesh patch exchange, peer merge
  (direct observation beats hearsay), RF-shadow evidence layer.
- `swarm.py` — beacons, leader election, wedge formation, sealed-bid
  delivery auction with load balancing, mission FSM (replicated on every
  robot — any survivor can finish the mission).
- `navigator.py` — potential-field go-to-goal with lidar repulsion and a
  stuck-escape wiggle. Teammates are avoided for free (they're on lidar).

## Honest limitations (a.k.a. the roadmap)

- **Shared map origin**: rovers deploy from a surveyed base, so maps merge
  by index. Real C-SLAM earns the transform via feature matching / inter-robot
  loop closures (see Swarm-SLAM, DOOR-SLAM, Kimera-Multi).
- **Idealized RF**: log-distance + penetration loss, i.i.d. shadowing. No
  multipath fading, no antenna patterns. The RSSI→range inversion is
  optimistic; obstructed links are gated rather than corrected.
- **Correlated estimates**: peer-to-peer range updates treat the peer's
  broadcast covariance as independent, which double-counts information
  (the classic cooperative-localization consistency problem). Pad anchors
  keep it bounded; a covariance-intersection update is the proper fix.
- **Yaw from a compass**: heading is magnetometer-style (absolute + noise),
  keeping yaw drift out of the mapping problem so position estimation stays
  the focus.
- **Reactive navigation**: potential fields can trap in deep concave
  obstacles; the arena is convex-ish by design. Swap in Nav2 per rover if
  you need real planning.
- **Next research step**: *active* link-informative motion — perturb
  formation slots to trade exploration coverage against expected RF
  information gain (make the squadron fly through shadows on purpose).
```
