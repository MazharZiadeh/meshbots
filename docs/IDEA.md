# Formation as Aperture
### The research I'm doing — in one page

## The observation

Robot teams that work where there's no infrastructure (disaster zones,
farms, other planets) route their own communications: each robot is a node
in an ad-hoc mesh, relaying packets for teammates out of range. The entire
robotics literature treats that mesh as *plumbing* — bytes go in, bytes
come out.

But a radio link is also a **measurement**. The signal strength (RSSI) of
every packet that arrives tells you something about the distance between
sender and receiver. And when a packet arrives *weaker* than that distance
predicts, something is standing between the two robots — the attenuation
itself is a crude X-ray of the environment.

So the idea: **the mesh the team already carries is a sensor it's throwing
away.** No new hardware. The measurements are free — they ride on the map
updates, status beacons, and task negotiations the robots were sending
anyway.

## What I built (proof of concept, all open-source, ROS 2 + Gazebo)

Three rovers doing a delivery mission with **no central coordinator**:
leader election with failover, wedge formation, market-based task auctions,
collaborative mapping where map chunks travel *only* over the mesh
(hop-by-hop through teammates — ~8,400 relayed packets per mission).

On top of that, communication-as-sensing:

1. **Localization from traffic.** Each robot's drift-prone odometry is
   corrected by range estimates inverted from the RSSI of packets it hears
   — from teammates (whose broadcast positions carry uncertainty) and from
   cheap transmit-only tags on the delivery pads. Measured across 8
   randomized missions: team position error drops **~30% (better in 8 of
   8 runs, paired)** versus the same system with the RF factors switched
   off.
2. **Mapping what no camera sees.** Links that lose more signal than
   free-space predicts paint "something is here" evidence along the line
   between the two robots — the mesh maps space no lidar has looked at.
   Lidar also gates the RF (rays through known walls aren't trusted for
   ranging), so the two senses correct each other.

## The actually-novel part

Everything above is engineering on known physics. The new question is:

> If links are sensors, then **where a robot stands changes what the team
> can measure**. The squadron's *formation shape* is a free variable the
> mission barely constrains. So treat the formation as a steerable sensing
> aperture — and plan it.

Each follower continuously evaluates small perturbations of its formation
slot and picks the one that maximizes *expected information from its
links*: predicted uncertainty reduction from the range measurements those
link geometries would produce (two crossing links beat two parallel ones —
same math as GPS satellite geometry), plus how much unexplored map the
links would sweep — minus a cost for deviating from the wedge, and never
letting the command link degrade below a floor. Sometimes a robot should
swing wide so its link to the leader crosses unmapped ground. Sometimes it
should *let a link go through a suspected obstacle on purpose*, because
the degradation is data.

The neighboring literatures each miss this: RF-mapping work (Mostofi et
al., UCSB) flies *dedicated* measurement trajectories — mapping is the
mission; cooperative-localization work uses *dedicated ranging hardware*
(UWB); "communication-aware planning" treats link quality strictly as a
*constraint to protect*, never a sensor to feed; and 6G "integrated
sensing and communication" (ISAC) does swarm beamforming physics, not
robot-team localization and occupancy mapping from mesh traffic. The
intersection — opportunistic sensing from mission traffic + formation
geometry planned to make that traffic informative, decentralized, under
mission constraints — appears to be open ground (systematic literature
check still in progress, so "appears").

## How I know if it works

The whole thing is falsifiable by construction: every experiment is a
paired A/B — same random seeds, fixed wedge vs. informative formation —
measuring localization error, map coverage, and *mission completion time*
(the cost side of the trade). The pipeline already killed one bad
configuration honestly (an early noise setting showed zero RF benefit, and
the numbers said so).

**And the first A/B verdict is in, honestly reported:** at the first
operating point the formation planner harvests measurably more RF
information (median 44% error correction vs 36% for the fixed wedge,
positive in 8/8 paired missions) — but the extra maneuvering integrates
enough additional odometry noise to consume the gain, and costs
mission-window completions. Net fused error: 0.66 vs 0.69 m. The
information is real; the price is currently too high. Tracing the
mission-vs-sensing Pareto front is the next experiment, one command per
point.

**Output goal:** arXiv preprint (cs.RO) + the open-source testbed. Total
budget so far: $0.

**Code:** https://github.com/MazharZiadeh/meshbots
