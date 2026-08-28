# LinkedIn launch post (draft)

*Post as plain text. Repo link in the body AND repeated as first comment.
Attach a visual: the 20s Gazebo/RViz screen recording if possible, else the
side-by-side map figure (docs/rf_shadow_sidebyside.png once generated), else
the one-pager PDF. Hashtags at the end, nothing mid-text.*

---

Robot teams that work without infrastructure carry a sensor they throw away:
their own radio mesh.

Every packet a robot relays for a teammate arrives with a measured signal
strength. That RSSI constrains the distance between the two robots — and
when a packet arrives weaker than that distance predicts, something is
standing between them. The attenuation is a crude X-ray. No new hardware;
the measurements ride on the map updates, beacons, and task auctions the
team was sending anyway.

I built and open-sourced a proof of concept: three simulated rovers running
a delivery mission with no central coordinator — leader election with
failover, market-based task allocation, and collaborative mapping whose map
chunks travel only over the mesh, hop-by-hop. On top of that,
communication-as-sensing:

- Localization from traffic: drift-prone odometry corrected by ranges
  inverted from packet RSSI. Across 24 paired randomized missions in three
  campaigns: better than the no-RF baseline in 23 of 24, median 40% less
  team position error. Zero additional hardware.
- Mapping what no camera sees: links that lose more signal than free space
  predicts paint obstruction evidence along the inter-robot ray — the mesh
  maps space no lidar has looked at.

The research question sitting on top: if links are sensors, the formation
is a free variable — so treat the squadron's shape as a steerable sensing
aperture and plan it. Each robot perturbs its formation slot to make its
own links informative: crossing links beat parallel ones (same geometry
logic as GPS dilution-of-precision), and a link swept through unmapped
space is a measurement. First paired A/B campaign of that planner, honestly
reported: it harvests measurably more RF information (median 44% correction
vs 36% for a fixed wedge, positive in 8 of 8 missions) — but the extra
maneuvering integrates enough additional odometry noise to consume the
gain, and costs mission time. The information is real; at this tuning the
price is too high. Tracing that trade-off curve is the next experiment,
and the harness makes each point one command.

Along the way the planner taught me something better than a benchmark win:
maximizing information gain amplified a consistency flaw in my estimator
(correlated peer updates → overconfidence → a locked-in bias that
deadlocked delivery docking). The fix is thematically satisfying — the
delivery pad's own radio chirp doubles as the docking sensor.

Everything is reproducible from one script — paired seeds, ablations built
into every run, and a limitations section that says "sim-only, idealized
RF" out loud. Built in the open.

Repo: https://github.com/MazharZiadeh/meshbots
Research brief + working paper draft are in /docs.

#robotics #ROS2 #multirobot #SLAM #ISAC

---

## Post-2 candidate (the failure autopsy, ~1 week later)
"My formation planner worked perfectly. That's exactly why my localization
filter broke." — walk through §4.1: information maximization vs. filter
consistency, the docking livelock, the pad-chirp fix. Engineers share
honest failure analyses more than benchmark wins.

## Post-3 candidate
Campaign-2 Pareto: how much mission time does a unit of sensing information
cost? Trust-radius sweep figure.
