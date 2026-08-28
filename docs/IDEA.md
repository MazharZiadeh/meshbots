# The mesh is also a sensor
### The research I'm doing — in one page

## The observation

Robot teams that work where there's no infrastructure (disaster zones,
farms, other planets) route their own communications: each robot is a
node in an ad-hoc mesh, relaying packets for teammates out of range. The
robotics literature mostly treats that mesh as *plumbing* — bytes in,
bytes out.

But a radio link is also a **measurement**. The signal strength (RSSI) of
every packet that arrives says something about the distance between
sender and receiver. When a packet arrives *weaker* than that distance
predicts, something is standing between the two robots. And — the part
that turned out to matter most — if a robot watches how those ranges
change as it drives, the ranges reveal **how wrong its own wheel odometry
is.** The measurements are free: they ride on the map updates, status
beacons and task negotiations the robots were sending anyway.

## What I built (open source, ROS 2 + Gazebo)

Three rovers doing a delivery mission with **no central coordinator**:
leader election with failover, wedge formation, market-based task
auctions, collaborative mapping where map chunks travel *only* over the
mesh (≈8,400 relayed packets per mission). On top of that,
communication-as-sensing, evaluated with a paired Monte Carlo protocol —
every mission runs four localization estimators on identical odometry
and identical packets, so every run is its own ablation.

1. **Localization from traffic.** Each robot's drift-prone odometry is
   corrected by ranges inverted from the RSSI of packets it hears — from
   teammates (whose broadcast positions carry uncertainty) and from cheap
   transmit-only tags on the delivery pads. Team position error drops
   **41 % (better in 8 of 8 seeded missions)** versus the same system with
   the RF factors switched off.
2. **The mesh calibrates the wheels.** Each rover carries a hidden 4–9 %
   wheel-scale bias. Adding that bias as a state the range factors can
   observe cuts the error by **a further 20 % (8 of 8)** and recovers the
   bias to a **1.1 % residual within one mission** — from packet RSSI
   alone, no encoder calibration, no external positioning.
3. **Mapping what no camera sees.** Links that lose more signal than
   free space predicts paint "something is here" evidence along the line
   between the robots. Lidar gates the RF in return (rays through known
   walls aren't trusted for ranging), so the two senses correct each other.

## The question I asked on top — and the honest answer

> If links are sensors, then **where a robot stands changes what the team
> can measure**. The squadron's formation shape is a free variable the
> mission barely constrains. So treat the formation as a steerable sensing
> aperture — and plan it.

Each follower evaluates small perturbations of its formation slot and
picks the one that maximizes expected information from its links (two
crossing links beat two parallel ones — the same geometry logic as GPS
satellites — plus how much unmapped space the links would sweep), minus
the cost of deviating and of moving.

**First paired A/B: negative.** The planner harvested measurably more RF
information but the extra manoeuvring cost as much odometry drift as it
bought, plus mission time. Digging into *why* produced the more useful
finding: an information-gain planner amplifies any inconsistency in the
estimator it feeds, and the safeguard I had added for that consistency
(a covariance floor) had silently removed the geometric signal the planner
depended on. **Second attempt**, with motion priced in the objective and
on the improved estimator: zero mission cost, no extra drift, modest gains
(coverage better in 7/8 seeds, error better in 5/8). Safe, not powerful —
and reported as such.

## Where it stands against the literature

Reading inter-robot RSSI for localization exists; RF-attenuation mapping
exists (Mostofi group); formation geometry optimized offline for UWB
observability exists (Cossette 2022, Ahmed 2024); range-aided odometry
scale calibration exists with UWB. What appears to be new is the
combination in one decentralized live-mission system, the **wheel-scale
calibration from mission-traffic RSSI specifically**, the online slot
perturbation with a map-coverage term on the link rays, and the paired
accounting of what sensing-driven motion costs a mission. The full
literature pass, with what is and is not new, is in `docs/RELATED.md`.

## What decides whether this becomes a paper

The estimator currently inverts the same propagation model the simulator
uses — so the calibration result shows the math is right, not yet that it
survives a wrong model. The experiment running now sweeps the channel
(2→8 dB shadowing, correlated fading, unknown per-device offsets, and a
deliberately wrong path-loss exponent). If the gain survives, the paper
exists; if it doesn't, that is the paper's honest result instead. Either
way the next step after that is a two-radio hardware measurement to
replace assumed channel numbers with measured ones.

**Output goal:** arXiv preprint (cs.RO) + the open-source testbed.

**Code and data:** https://github.com/MazharZiadeh/meshbots
