# Related work — targeted literature pass (2026-08-29)

Scope: the closest prior work for each claim in `PAPER.md` / `IDEA.md`.
All entries are real papers found via web search in this pass; where I could
only confirm an abstract (not full text) I say so. All items flagged for
verification in the first pass were resolved in a second pass (2026-08-29)
against primary text where obtainable; each now carries either a confirmed
citation or an explicit "could not confirm — do not cite". Nothing here was
invented.

Notation for the "Δ" line: *what they do* → *how we differ / what it means for
the novelty claim*.

---

## 1. Mostofi group (UCSB): RF obstacle mapping, see-through imaging, informative wireless-measurement paths

1. **Mostofi & Gonzalez-Ruiz, "Compressive cooperative obstacle mapping in mobile networks," MILCOM 2010** (invited).
   - Two robots make WiFi power measurements across a region; compressive sensing recovers an obstacle map from the attenuation.
   - Δ: mapping *is* the mission and measurements are dedicated scans; centralized reconstruction. Our RF-shadow evidence is the same physics (link-line excess attenuation → obstacle votes) applied to traffic sent for another reason, decentralized and online. **This is the direct ancestor of our mapping ingredient; cite it as such.**

2. **Mostofi, "Cooperative wireless-based obstacle/object mapping and see-through capabilities in robotic networks," IEEE Trans. Mobile Computing 11(5), 2012.**
   - Journal version: RSS-based tomographic mapping with robot pairs, compressive reconstruction, see-through of occluded areas.
   - Δ: same as above. Our contribution is not the mapping principle.

3. **Gonzalez-Ruiz & Mostofi, "Cooperative robotic structure mapping using wireless measurements — a comparison of random and coordinated sampling patterns," IEEE Sensors J. 13(7), 2013.**
   - Studies *which robot trajectories/sampling patterns* make wireless measurements informative for structure mapping (random vs coordinated).
   - Δ: this is the closest Mostofi work to "planning motion for informative RF measurement." It is trajectory design for a dedicated mapping mission, not perturbation of a mission-constrained formation. We must cite it and state the difference precisely.

4. **Gonzalez-Ruiz, Ghaffarkhah & Mostofi, "An integrated framework for obstacle mapping with see-through capabilities using laser and wireless channel measurements," IEEE Sensors J. 14(1), 2014.**
   - Fuses laser and wireless attenuation into one occupancy estimate: laser maps the visible parts (occupancy grid), wireless channel measurements map the occluded parts (confirmed from the IEEE Xplore abstract, DOI 10.1109/JSEN.2013.2278394). A secondary summary also attributes to it an "adaptive path planning strategy that uses the current uncertainty estimate to collect more informative wireless measurements" — **could not confirm against the primary text (paywalled; no abstract via Crossref) — do not cite this paper for adaptive planning until read.** The informative-sampling claim is safely citable from Gonzalez-Ruiz & Mostofi, IEEE Sensors J. 2013 (entry 3) instead.
   - Δ: laser+RF fusion is *already here*, and informative RF sampling patterns are in entry 3. Our "lidar gates RF, RF informs map" is a bidirectional variant, but the fusion idea is not new. The novelty must rest on (a) opportunistic traffic, (b) formation-slot perturbation under mission constraints, (c) decentralization — not on laser+RF fusion or informative-path planning per se.

5. **Depatla, Buckland & Mostofi, "X-ray vision with only WiFi power measurements using Rytov wave models," IEEE Trans. Veh. Tech. 2015.** — Wave-model (not just LOS-attenuation) imaging through walls. Δ: shows where a serious RF-mapping paper goes beyond the log-distance model; our channel simulator is far cruder — a limitation to state.

6. **Karanam & Mostofi, "3D through-wall imaging with unmanned aerial vehicles using WiFi," IPSN 2017**; **Depatla, Karanam & Mostofi, "Robotic through-wall imaging," IEEE Antennas & Propagation Mag. 2017.** — Two drones, one Tx one Rx, dedicated flight paths; MRF + loopy BP + sparse recovery. Δ: again dedicated trajectories. Cite for completeness.

7. **Malmirchegini & Mostofi, "On the spatial predictability of communication channels," IEEE Trans. Wireless Comm. 11(3), 2012.** — Probabilistic channel prediction (path loss + shadowing + multipath) at unvisited locations from sparse samples. Δ: gives the correct statistical decomposition we do *not* model (multipath); useful to cite when describing our idealized channel.

8. **Ghaffarkhah & Mostofi, "Communication-aware motion planning in mobile networks," IEEE TAC 2011**; **Ghaffarkhah & Mostofi, "Decentralized communication-aware motion planning in mobile networks: an information-gain approach," J. Intell. Robot. Syst. 2009**; **Yan & Mostofi, "Robotic router formation in realistic communication environments," IEEE T-RO 28(4), 2012**; **Yan & Mostofi, "Co-optimization of communication and motion planning…," IEEE TWC 2013.**
   - Co-optimize sensing/communication/motion; the 2009 paper already uses an *information-gain* objective in a decentralized setting; the 2012 T-RO paper chooses robot positions (router formation) for link quality.
   - Δ: in all of these the link is either a constraint or a throughput objective; the *sensing* objective is about an external target, not about the link itself as a sensor. This is exactly the "constraint side vs objective side" distinction PAPER.md draws — but be careful: Mostofi's framing is "co-optimization," not pure constraint, so the sentence "the link is always on the constraint side" is too strong. Reword to "the link's *quality* is optimized, but its *measurement* is not treated as a sensor of geometry/occupancy."

9. **Muralidharan & Mostofi, "Communication-aware robotics: exploiting motion for communication," Annual Review of Control, Robotics, and Autonomous Systems 4, 2021** (review). — Survey of the whole line. Good single citation for the group.

## 2. Range-aided odometry and wheel-scale / slip calibration

10. **Borenstein & Feng, "Measurement and correction of systematic odometry errors in mobile robots," IEEE T-RA 12(6), 1996 (UMBmark).**
    - Dominant systematic errors are wheelbase uncertainty and unequal wheel diameters; the procedure detects diameter differences down to 0.1 % and calibrates scale to ≈0.3–0.5 % of full scale; ≥10× odometric accuracy improvement.
    - Δ: offline, needs a surveyed square path. Our EKF scale state is the online version. Cite for what "wheel scale bias" means and its magnitude.

11. **Censi, Franchi, Marchionni & Oriolo, "Simultaneous calibration of odometry and sensor parameters for mobile robots," IEEE T-RO 29(2), 2013.**
    - Max-likelihood, closed-form calibration of wheel radii + wheelbase + sensor extrinsics from exteroceptive (laser) data, arbitrary trajectories.
    - Δ: uses a metric exteroceptive sensor, not ranges. Establishes that "estimate odometry intrinsics from any drift-free exteroceptive cue" is standard.

12. **De Giorgi, De Palma & Parlangeli, "Online odometry calibration for differential drive mobile robots in low traction conditions with slippage," Robotics 13(1):7, 2023.**
    - Online calibration using encoder/gyro/IMU redundancy, explicitly handling slip. Δ: no ranging; closest "online scale calibration" for diff-drive.

13. **Lee, Chung & Yoo, "Tightly-coupled LiDAR-IMU-wheel odometry with online calibration of a kinematic model for skid-steering robots," arXiv 2404.02515, 2024** and the follow-up neural kinematic model (arXiv 2407.08907). — Online wheel-kinematic-parameter estimation inside a factor graph, LiDAR as the reference. Δ: same pattern, LiDAR instead of ranging.

14. **Nguyen, Zaidi & Xie, "Loosely-coupled UWB-aided scale correction for monocular visual odometry," Unmanned Systems 2020**; **Nguyen et al., "Tightly-coupled UWB-aided monocular visual SLAM with degenerate anchor configurations," Autonomous Robots 2020.**
    - Estimate the *metric scale* of monocular VO from UWB ranges to (even a single) anchor.
    - Δ: this is the nearest existing instance of "ranges observe an odometry scale factor." They do it for VO scale with UWB (σ ≈ 0.1 m); we do it for *wheel* velocity scale with RSSI (σ ≈ metres). The estimation structure is the same; what is new is the sensor (opportunistic RSSI) and the state (wheel scale), not the idea. **Do not claim "mesh calibrates the wheels" as a structurally new observation model.**

15. **Sun et al., "CT-UIO: continuous-time UWB-inertial-odometer localization using non-uniform B-spline with fewer anchors," IEEE TMC 2025 (arXiv 2502.06287).** Full text checked: the odometer scale is **not** estimated online — "the IMU bias and odometer scale are initialized" beforehand, and in experiments "the odometer scale factor error is set to be 0.5 %" (Turtlebot3 Waffle Pi, Nooploop LinkTrack UWB, LOS ranging error N(−0.055 m, 0.051² m²)). Positioning 0.15–0.40 m. Δ: confirms (a) a realistic wheel-scale error magnitude (0.5 %) used by a 2025 UWB+odometer paper and (b) that this line calibrates scale *offline*; do not cite it as prior art for online scale estimation. Electronics 13(8):1518 (2024, dynamic UKF UWB/wheel odometry) was not checked — cite only as "UWB+odometer tightly-coupled fusion".

16. **Yang et al., "Robust online calibration for UWB-aided visual-inertial navigation with bias correction," arXiv 2508.10999, 2025.** — Online UWB range bias/scale calibration inside VIO. Δ: shows the field also treats the *range* as needing scale/bias calibration — relevant because our RSSI range model (P_tx, PL_0, n) is assumed known.

17. **Zhang, Shan et al., "GPS-aided visual wheel odometry," IROS 2023 (arXiv 2308.15133).** — MSCKF with wheel encoders + GPS, online extrinsic calibration. Δ: GPS is the absolute reference; we use peer/tag ranges. Same estimator family.

## 3. RSSI-based cooperative / multi-robot localization; how bad RSSI ranging really is

18. **Oliveira, Li, Almeida & Abrudan, "RSSI-based relative localisation for mobile robots," Ad Hoc Networks 13, 2014.**
    - Anchor-less relative localization of a robot team from **RSSI of the messages the nodes already exchange** (Kalman-smoothed pairwise "signal distances" → Floyd–Warshall → MDS).
    - Δ: **This is prior art for "RSSI from ordinary traffic as ranging" in multi-robot teams.** It is coarse (they call it "coarse relative localisation suitable for coordination"), not fused with odometry in an EKF, no mapping, no formation planning. Our claim (i) must be stated as "we harvest per-packet RSSI as *EKF range factors* fused with odometry and anchors," citing Oliveira as the precedent for the opportunistic-traffic idea.

19. **Zickler & Veloso, "RSS-based relative localization and tethering for moving robots in unknown environments," ICRA 2010.** — LANdroids: infer distance from an RSSI trace + odometry, no anchors. Δ: single-target tethering; again shows RSSI+odometry localization is old.

20. **Rubenstein, Ahler, Hoff, Cabrera & Nagpal, "Kilobot: a low cost robot with scalable operations designed for collective behaviors," Robotics & Autonomous Systems 62(7), 2014** (orig. ICRA 2012).
    - Kilobots measure neighbour distance from the **received intensity of the IR communication signal itself** — communication-as-ranging is built into the platform, used for localization/formation in a 1000-robot swarm.
    - Δ: strongest "the exact framing already exists" precedent, at swarm scale, in IR rather than RF. Our framing "the mesh is a sensor being thrown away" is true of the *ROS 2 / ad-hoc-WiFi robotics literature*, not of swarm robotics. **Say so.**

21. **Latif & Parasuraman, "Multi-robot synergistic localization in dynamic environments (MRSL)," arXiv 2206.03573, 2022**; **Latif & Parasuraman, "HGP-RL: distributed hierarchical Gaussian processes for Wi-Fi-based relative localization in multi-robot systems," IROS 2024**; **Sagale, Kargar Tasooji & Parasuraman, "DCL-Sparse: distributed range-only cooperative localization…," arXiv 2412.14793, 2024.**
    - Decentralized multi-robot localization from WiFi RSSI (to a common AP, or robot-to-robot), Bayesian fusion, GP RSSI maps; DCL-Sparse handles sparse range graphs.
    - Δ: closest *contemporary* group. They use WiFi RSSI from communication hardware but (per abstracts) not explicitly per-packet mission traffic; no occupancy mapping from attenuation; no formation planning. Cite all three; position ours as adding mapping + planning + traffic-coupled schedule.

22. **Hernandez-Martinez et al., "Multi-robot formation based on RSSI power level and radiation pattern," Int. J. Systems Science 53(3), 2022.** — Formation/dispersion control using RSSI as the distance sensor between robot pairs, accounting for antenna radiation pattern. Δ: RSSI-as-sensor *for formation control*; the inverse of ours (we shape the formation to improve the RSSI sensing). Also a reminder that antenna patterns matter (we ignore them).

23. **Ferris, Fox & Lawrence, "WiFi-SLAM using Gaussian process latent variable models," IJCAI 2007.** — Fingerprint-style WiFi SLAM. Δ: infrastructure APs, not peers; cite as the root of WiFi-RSSI robot localization.

24. **Twigg, Fink, Yu & Sadler, "RSS gradient-assisted frontier exploration and radio source localization," ICRA 2012**; **Fink & Kumar, "Online methods for radio signal mapping with mobile robots," ICRA 2010.** — ARL/Penn line: robots measure RSS of their own network links to map and localize the radio field. Δ: "measure your own links" is present here, but the target is the *radio map / source*, not team pose or occupancy.

25. **Bullmann, Fetzer, Ebner, Ebner, Deinzer & Grzegorzek, "Comparison of 2.4 GHz WiFi FTM- and RSSI-based indoor positioning methods in realistic scenarios," Sensors 20(16):4515, 2020.**
    - Real building, commodity APs: log-distance exponent γ ∈ [2.5, 3.7] per AP, fitted shadowing σ = 5.5 dB; RSSI positioning mean error 4.3–6.4 m (LS 5.1–6.4 m, PF 4.3–4.9 m); calibrated FTM 3.3–4.4 m.
    - Δ: **numbers to reuse** (see §Numbers). Our σ_dB and resulting ~0.7 m fused ATE will look optimistic against this unless the simulation σ_dB ≥ 5 dB is stated explicitly.

26. **Wang, Zhang, Wu & Guo, "Rethinking RSSI for WiFi sensing," npj Wireless Technology 2, 2026** (also arXiv 2602.14004). — Shows RSSI carries Doppler/AoA/delay cues with a 3-antenna Rx; also states plainly that RSSI is "often regarded as too coarse for sensing," with displacements >1 m needed. Δ: cite when defending RSSI (vs CSI) as the cheapest possible modality and when listing CSI as the upgrade path.

27. **Dijkstra, Jadhav, Sloot, Marcantoni, Jayawardhana, Gil & Haghighat, "WiFi-CSI sensing and bearing estimation in multi-robot systems: an open-source simulation framework," arXiv 2410.01398, 2024** (and the Gil-group WSR toolbox it replicates).
    - Bearing between robots from the CSI of *inter-robot communication packets*, Gazebo + Turtlebot3. The WSR toolbox it replicates is confirmed: **Jadhav, Wang, Zhang, Khatib, Kumar & Gil, "A wireless signal-based sensing framework for robotics," IJRR 41(11–12):955–992, 2022 (arXiv 2012.04174)** — AOA to other robots from WiFi CSI using the robot's own motion as a virtual antenna array, NLOS, no infrastructure.
    - Δ: **another "communication packets as sensing" precedent, in the same simulator ecosystem**, with bearing instead of range. Must be cited; positions our RSSI choice as the coarser, hardware-free end of the same idea.

## 4. Communication-aware planning; formation geometry optimized for range-only localization

28. **Fink, Ribeiro & Kumar, "Robust control for mobility and wireless communication in cyber-physical systems with application to robot teams," Proc. IEEE 100(1), 2012.** — Canonical connectivity-as-constraint mobility control. Δ: what PAPER.md calls "link on the constraint side."

29. **Mikkelsen, Galeazzi & Fumagalli, "Optimal multi-robot communication-aware trajectory planning by constraining the Fiedler value," 2024 (arXiv 2406.18452)**; distributed ADMM version arXiv 2408.05111. — Fiedler-value constraint in trajectory optimization. Δ: this is the `[FiedlerPlanning24]` placeholder in PAPER.md.

30. **Zhou & Roumeliotis, "Optimal motion strategies for range-only constrained multisensor target tracking," IEEE T-RO 24(5):1168–1185, 2008**; **Zhou & Roumeliotis, "Multirobot active target tracking with combinations of relative observations," IEEE T-RO 27(4):678–695, 2011** (venues/years confirmed via dblp/IEEE Xplore). Both are one-step-ahead (myopic) *online* motion selection each time step; the target is an external moving object, not the team itself, and there is no mapping term.
    - Robots choose next positions to make their *range* measurements to a target most informative (min trace / max eigenvalue of covariance), proven NP-hard with speed limits, relaxations given.
    - Δ: our `loc_gain` (predicted scalar-EKF variance reduction per link, direction diversity rewarded) is a one-step greedy instance of exactly this. Cite; do not claim the objective is new.

31. **Le Ny & Chauvière, "Localizability-constrained deployment of mobile robotic networks with noisy range measurements," ACC 2018 (arXiv 1801.04816).**
    - **Full text checked.** Robots descend the gradient of a potential f(p) = f_loc(p) + α·f_conn(p) + β·f_task(p), where f_loc is a CRB/Fisher-information localizability term (D-, A-, T-, E-optimal variants), f_conn a connectivity term and f_task a generic task potential (their example: reach target x-coordinates). Distributed gradient computation is discussed; gradient descent runs continuously during deployment (reactive, not a precomputed formation), but validation is **simulation only** (4 robots, additive range noise σ = 0.1). No coverage/occupancy/mapping term.
    - Δ: closest prior work to the *online* half of our planner: information potential + task potential, descended online. Ours differs by the map term on link rays, the opportunistic RSSI sensor, the explicit mission constraints (PDR floor, obstacle-free slot), and a live mission. Cite it as the direct precedent for "loc_gain + task cost, online".

32. **Cossette, Shalaby, Saussié, Le Ny & Forbes, "Optimal multi-robot formations for relative pose estimation using range measurements," IROS 2022 (arXiv 2205.14263)**; **Ahmed, Shalaby, Le Ny & Forbes, "Optimal robot formations: balancing range-based observability and user-defined configurations," IROS 2024 (arXiv 2403.00988).**
    - Optimize formation geometry by maximizing Fisher information of inter-agent ranges (UWB); the 2024 paper adds a *user-defined desired formation* term and trades it against observability — i.e. **the same mission-vs-sensing trade our (ρ, λ, α) parameterize**, with hardware experiments.
    - **Full text checked (both).** Cossette 2022: "the optimization is only done offline … precompute optimal formations for varying robot numbers N … store the solutions in memory onboard each robot"; a "distributed, real-time implementation … in the presence of obstacles … is beyond the scope of this paper." Two UWB tags per quadcopter (~10 cm ranging), Fisher-information cost + collision term; experiment: 3 quadcopters, formation change gives a 68 % reduction in estimation variance (mean positioning error 0.22 m). Ahmed 2024: "the optimization is done offline. These formation results can then be stored in the memory of the robots and used for online planning"; cost J_cov = J_adj + J_overlap + J_est + J_col, where the coverage terms are **camera field-of-view footprint** terms (adjacency/overlap of sensor discs), *not* anything defined on the ranging rays; evaluated in coverage path planning with EKF-SLAM, simulation + experiment.
    - Δ: **This is the closest prior work to contribution (iv).** Differences we can now state precisely: (1) both optimize the formation *offline* and track it; ours re-evaluates slot perturbations every planning cycle online; (2) neither has any term on the inter-robot link rays — our `map_gain` (unknown cells swept by the link segment) has no counterpart; (3) they use dedicated UWB, we use opportunistic RSSI; (4) the short-vs-long link tension between ranging and mapping does not arise for them. Ahmed's "user-defined formation vs observability" trade is the precedent for our mission-vs-sensing trade and must be cited as such. "Formation as aperture" as a *name* is ours; as a *concept* it is already in Cossette/Ahmed and Le Ny.

33. **Knowles, Dai & Gao, "Multi-robot collaborative localization and planning with inter-ranging," ION GNSS+ 2024 (arXiv 2406.16679).** — CADRE-style lunar rovers, UWB mesh ranging + VO; decentralized planner picks paths to lower a geometry-based (DOP-like) uncertainty metric; hardware demo. Δ: the `[InterRanging24]` placeholder. Same "move to make ranges informative" idea, dedicated UWB.

34. **Quattrini Li, Penumarthi, Banfi, Basilico, O'Kane, Rekleitis, Nelakuditi & Amigoni, "Multi-robot online sensing strategies for the construction of communication maps," Autonomous Robots 44, 2020.** — Robots plan where to measure to build a *communication map* (RSS field) online. Δ: information-driven placement for RF measurement, but the product is a comm map, not occupancy/pose.

35. **Clark, Hsu et al., "PropEM-L: radio propagation environment modeling and learning for communication-aware multi-robot exploration," RSS 2022 (arXiv 2205.01267).** — Uses the robots' *3D geometric map* to predict RSS (LOS/wall attenuation), six-robot SubT team. Δ: exactly the reverse direction of our "RF informs map" (map informs RF). Good pairing citation for the bidirectional lidar↔RF coupling; note that our "lidar gates RF" half is already done here.

## 5. ISAC applied to swarms; "communication as sensing" in robotics

36. **Zhou, Leng, Wang & Liu, "Integrated sensing and communication in UAV swarms for cooperative multiple targets tracking," IEEE Trans. Mobile Computing 22(11), 2023.** — ISAC waveforms on UAVs track external targets. Δ: PHY-level, external targets; no self-localization/mapping.

37. **Zhai, Ni, Wang, Niyato & Hossain, "Integrated sensing and communication with UAV swarms via decentralized consensus ADMM," arXiv 2511.03283, 2025.** — Swarm positions optimized (consensus ADMM) to balance uplink rate vs sensing CRB of a virtual array. Δ: the `[UAVSwarmISAC25]` placeholder; geometry–sensing coupling is here, but for beamforming at external targets, not team pose or occupancy.

38. **Gounis, Tegos, Tyrovolas, Diamantoulakis & Karagiannidis, "When SLAM meets wireless communications: a survey," arXiv 2602.06995, 2026**; **Sansoni-era "Sensing with mobile devices through radio SLAM," arXiv 2509.07775, 2025 (Gounis-group / radio-SLAM community)**; **"Cooperative mapping, localization, and beam management via multi-modal SLAM in ISAC systems," arXiv 2507.05718, 2025.** — 6G radio-SLAM: mmWave multipath components as landmarks. Δ: confirms the ISAC community's "sensing" means channel-parameter estimation at 5G/6G PHY, not occupancy grids from mesh RSSI. Our "robotics-grade ISAC" positioning is defensible, but the survey explicitly flags RF as a scale proxy for monocular SLAM — adjacent to our wheel-scale idea.

39. **Kim, Zalat, Bahoo & Saeedi, "Structure from WiFi (SfW): RSSI-based geometric mapping of indoor environments," ACC 2024 (arXiv 2403.02235)**; **Kim, Lisondra, Bahoo & Saeedi, "Inverse k-visibility for RSSI-based indoor geometric mapping," Autonomous Robots 2026 (arXiv 2408.07757).** — A single robot builds free-space maps from RSSI of *ordinary WiFi APs* using visibility geometry; compared to LiDAR ground truth. Δ: occupancy/free-space from RSSI without dedicated Tx — the opportunistic half of our mapping claim exists for infrastructure APs; ours uses peer robots and tags.

40. **Ramachandran & Berman, "Automated construction of metric maps using a stochastic robotic swarm leveraging received signal strength," IEEE T-RO 2019 short paper.** — Swarm + RSS from external transmitters → free-space density → topological thresholding. Δ: RSS from fixed external Tx, offline processing.

## 6. Consistency / covariance intersection in cooperative localization

41. **Julier & Uhlmann, "A non-divergent estimation algorithm in the presence of unknown correlations," ACC 1997.** — Covariance intersection; consistent fusion under unknown cross-correlation. Δ: the principled fix we cite; our inflation+floor is a crude surrogate.

42. **Roumeliotis & Bekey, "Distributed multirobot localization," IEEE T-RA 18(5), 2002.** — Shows the centralized CL EKF decomposes into communicating per-robot filters *with* tracked cross-covariances. Δ: our scalar per-robot EKF drops the cross-covariance entirely; this is why §4.1 happens.

43. **Bahr, Walter & Leonard, "Consistent cooperative localization," ICRA 2009.** — Bank of filters tracking measurement origins to prevent re-use → conservative covariance in AUV moving-baseline ranging. Δ: earliest explicit "avoid information re-use in range-only CL"; the docking livelock in §4.1 is a textbook case of the failure they prevent.

44. **Carrillo-Arce, Nerurkar, Gordillo & Roumeliotis, "Decentralized multi-robot cooperative localization using covariance intersection," IROS 2013.** — O(N) CI-based CL, provably consistent, asynchronous. Δ: exactly the drop-in replacement for our peer factor.

45. **Luft, Schubert, Roumeliotis & Burgard, "Recursive decentralized localization for multi-robot systems with asynchronous pairwise communication," IJRR 37(10), 2018.** — Approximate pairwise cross-correlation tracking, resources independent of team size. Δ: the other principled option.

46. **Chang, Chen & Mehta, "Resilient and consistent multirobot cooperative localization with covariance intersection," IEEE T-RO 2021 (arXiv 2108.08789).** — CI-based CL that separates communication from observation updates; stays consistent under blocked comms. Δ: relevant because our factor schedule is *tied to comms traffic* — their decoupling is what we would need.

47. **Huang, Mourikis & Roumeliotis, "Analysis and improvement of the consistency of EKF-based SLAM" (FEJ-EKF), 2008–2010**; **Bailey, Nieto, Guivant, Stevens & Nebot, "Consistency of the EKF-SLAM algorithm," IROS 2006.** — Spurious information gain along unobservable directions → overconfidence. Δ: background for "more factors collapsed covariance faster than error."
   - **On contribution (v)** ("active information-gain planning amplifies estimator inconsistency"): I found no paper stating this interaction at the multi-robot system level in these searches; the closest are the active-SLAM consistency literature (e.g., "The SLAM confidence trap," Sansoni & Tosetti, arXiv 2602.15884, 2026, which argues for consistency as a primary metric but does not treat active planning) and the general double-counting literature above. The observation is plausible-but-known-in-folklore; **claim it as an empirical system-level report, not a discovery.** Reviewers from the CL community will regard it as an expected consequence of dropping cross-covariances.

## 7. Work that uses mission/communication traffic opportunistically for sensing (the exact framing)

Ranked by closeness to our framing:

- **Kilobot (Rubenstein et al. 2014)** — IR communication intensity *is* the range sensor; formation/localization built on it. Swarm-scale precedent.
- **Oliveira et al. 2014** — RSSI of exchanged messages → relative localization of a robot team. Direct precedent for "opportunistic traffic RSSI as range."
- **WSR / Dijkstra et al. 2024 (Gil group)** — CSI of inter-robot packets → bearing, in Gazebo/ROS. Same idea, better observable.
- **Latif & Parasuraman 2022–2024** — WiFi RSSI, decentralized multi-robot localization.
- **Fink/Twigg (ARL) 2010–2012** — robots measure their own links to map RF.
- **Mostofi 2010–2017** — RSS attenuation between robots → obstacle map (dedicated scans).
- **Kim/Saeedi 2024–2026** — ordinary AP RSSI → free-space map.

I found **no** paper that combines (a) per-packet RSSI of mission traffic, (b) as both EKF range factor *and* occupancy evidence, (c) with wheel-scale as an EKF state, (d) decentralized, (e) with mission-constrained formation perturbation and a map-coverage term. The combination appears open; every ingredient has a precedent.

---

## Honest novelty statement

**Genuinely new (defensible):**
1. The *combination* in one decentralized ROS 2 system: mission-traffic RSSI → range factors + RF-shadow occupancy evidence + wheel-velocity-scale state, with lidar↔RF bidirectional gating, on a live mission with formation, auctions and delivery.
2. The `map_gain` term (rays swept through unknown cells) inside a formation-slot objective, and the explicit *ranging-prefers-short-links vs mapping-prefers-long-links* tension. I found no formation-optimization paper with an occupancy-coverage term on the link rays.
3. Online, per-cycle, hysteretic slot perturbation inside a live mission. **Confirmed against full texts:** Cossette 2022 and Ahmed 2024 optimize formations offline and store them; Le Ny & Chauvière 2018 is online gradient descent but simulation-only with a generic task potential and no map term; Zhou & Roumeliotis 2008/2011 are online but track an external target. Ours is the only one of these that (a) runs online, (b) inside a mission with delivery/auction constraints, (c) with a link-ray coverage term. Claim exactly that, no more.
4. The paired A/B mission-cost accounting (delivery time / completions vs information gained), and the honest negative-ish result.
5. Wheel-scale bias observed through *RSSI* ranges specifically (novel sensor for a known state).

**Applied / combined (must be cited, not claimed):**
- RSS-attenuation obstacle mapping and laser+RF fusion with uncertainty-driven measurement planning: Mostofi 2010–2014.
- RSSI of exchanged messages as range for team localization: Oliveira 2014; Kilobot 2014; Latif & Parasuraman.
- Formation geometry optimized for range-only estimation, including a mission-vs-observability trade: Zhou & Roumeliotis 2008/2011, Le Ny & Chauvière 2018, Cossette 2022, Ahmed 2024, Knowles 2024.
- Ranges observing an odometry scale: Nguyen et al. 2020 (UWB→VO scale); UWB+odometer scale-state fusion (2024–2025).
- Double counting / inconsistency and CI as the fix: Julier & Uhlmann 1997 → Carrillo-Arce 2013 → Luft 2018 → Chang 2021.

**Rewrite needed in PAPER.md §2:** "communication-aware planning treats the link as a constraint to protect, never a sensor to read" is false as written — Mostofi's co-optimization and Fink/Twigg's RSS mapping *read* the link; the Gil group reads CSI of inter-robot packets. Narrow the claim to: "no prior planner treats the *formation slot* as a variable to be perturbed for link-derived range+occupancy information under mission constraints."

## Numbers to reuse

| Quantity | Value | Source |
|---|---|---|
| Log-normal shadowing σ, indoor, by building type (primary table) | Grocery store 914 MHz: n = 1.8, σ = 5.2 dB; Retail store 914 MHz: n = 2.2, σ = 8.7 dB; Office, hard partitions 1500 MHz: n = 3.0, σ = 7.0 dB; Office, soft partitions 900 MHz: n = 2.4, σ = 9.6 dB; Office, soft partitions 1900 MHz: n = 2.6, σ = 14.1 dB; Factory LOS 1300 MHz: n = 1.6–2.0, σ = 3.0–6.0 dB | **Andersen, Rappaport & Yoshida, "Propagation measurements and models for wireless communications channels," IEEE Communications Magazine 33(1):42–49, 1995, Table 2** (read from the PDF; this is the table reproduced as Table 4.6 in Rappaport, *Wireless Communications: Principles and Practice*, 2nd ed., 2002). Cite the 1995 paper as primary. |
| Shadowing σ, multi-floor office building, 914 MHz | 5.8 dB overall (as low as ≈4 dB for individual areas) | Seidel & Rappaport, "914 MHz path loss prediction models for indoor wireless communications in multifloored buildings," IEEE Trans. Antennas Propag. 40(2):207–217, 1992 (abstract) |
| Shadowing σ, 2.4 GHz WLAN classrooms/labs/corridors | "3.3–4.1 dB" figures seen in a search summary — **could not confirm against a primary (the likely source, D. Tummala, "Indoor propagation modeling at 2.4 GHz for IEEE 802.11 networks," M.S. thesis, Univ. North Texas, 2005, is behind a bot-check; Faria's Stanford TR was unreachable) — do not cite these numbers.** Use Bullmann 2020 (5.5 dB) for 2.4 GHz WiFi instead. |
| Shadowing σ, indoor multi-room WiFi | 5.5 dB (fitted) ; γ ∈ [2.5, 3.7] | Bullmann et al., Sensors 2020 |
| Shadowing σ, multi-floor with FAF (433 MHz LoRa, 4-storey university building) | σ = 6.93 dB, n = 2.53, FAF 5.52 dB/floor | "Measurement-based modeling of large-scale and time-varying small-scale fading for LoRa in indoor multi-floor environments," *Sensors* 26(4):1152, 2026, DOI 10.3390/s26041152 (identity confirmed; numbers from abstract). **Note: 433 MHz LoRa, not 2.4 GHz WiFi** — cite only as "same order of magnitude across bands". |
| Shadowing σ, outdoor | 4–12 dB (typ. 8–10 dB) | same overviews |
| RSSI positioning error, real building, commodity APs | 4.3–6.4 m mean (vs 3.3–4.4 m calibrated FTM) | Bullmann et al. 2020 |
| Derived RSSI range σ_d at d = 10 m, n = 2.5, σ_dB = 5.5 | σ_d ≈ d·ln10·σ_dB/(10n) ≈ 5.1 m (i.e. ~50 %) — at d = 3 m ≈ 1.5 m | your own formula (PAPER §3) + Bullmann parameters |
| Wall attenuation (single obstruction on LOS link) | 10 dB per (virtual) wall | Latif & Parasuraman 2022, full text: "Each wall will attenuate 10 dBm of RSSI signal power" — this is a **simulation assumption**, not a measurement; do not cite it as an empirical number. Their sim result: ≈1.8 m localization error over a 3600 m² region, 35 % RMSE reduction vs baseline. For measured wall loss use a propagation source (Andersen et al. 1995 partition-loss discussion; or in-building partition loss measurements at 2.5 GHz, arXiv 1701.03415). |
| UWB LOS ranging accuracy | decimetre-level; NLOS adds unknown positive bias (tens of cm to m) | Cano/Le Ny IMM-NLOS 2020 (arXiv 2009.03538); CADRE/Knowles 2024 |
| Wheel-diameter mismatch detectable / scale calibration precision | 0.1 % ; 0.3–0.5 % of full scale | Borenstein & Feng 1996 |
| Odometry improvement from systematic calibration | ≥ 10× (Borenstein & Feng 1996, confirmed). The "up to 35× under slip" figure attributed to De Giorgi et al. 2023 **could not be confirmed** (MDPI/IRIS full text blocked; abstract confirms method — encoder/gyro/IMU redundancy to detect slip during online calibration — but gives no factor) — **do not cite the 35× number.** | Borenstein & Feng 1996; De Giorgi, De Palma & Parlangeli, *Robotics* 13(1):7, 2023 (method only) |
| Wheel/odometer scale-factor error assumed in a 2025 UWB+odometer system | 0.5 % | Sun et al., CT-UIO, IEEE TMC 2025 (full text) |
| UWB+IMU+odometer fused accuracy | 0.15–0.40 m | CT-UIO, TMC 2025 |

Sanity check against our sim: with σ_dB matching real indoor values (≥5 dB), single-link RSSI range σ at 5–10 m is 2.5–5 m. A 34 % ATE reduction to 0.68 m from such factors is only credible with many factors, short links (< 5 m), and strong anchors — say this explicitly and report the σ_dB used.

## Risks (claims a reviewer would reject)

1. **"Never a sensor to read" / "thrown away."** Directly contradicted by Oliveira 2014, Kilobot, Gil-group WSR/CSI, Fink/Twigg. Rewrite (see above).
2. **"Formation as aperture" as core novelty.** Cossette 2022 / Ahmed 2024 / Le Ny 2018 / Zhou & Roumeliotis 2011 already optimize formation geometry for range-only estimation, including a mission trade-off, with hardware. Novelty must be narrowed to the map term, opportunistic RSSI, online slot perturbation, and the mission-cost result.
3. **Laser+RF fusion and informative RF paths.** Gonzalez-Ruiz, Ghaffarkhah & Mostofi 2014 already do both. Claiming "lidar informing RF, RF informing map" as new will be rejected; claim it as the bidirectional gating in a decentralized online system.
4. **Contribution (v) as "first report."** CL reviewers will call it a known consequence of ignoring cross-correlation (Roumeliotis 2002, Bahr 2009, Carrillo-Arce 2013). Present as an empirical system-level cautionary result; implement CI (Carrillo-Arce) as the fixed arm, not inflation+floor, or the reviewer will ask why.
5. **RSSI ranging realism.** i.i.d. log-normal, no multipath/antenna pattern/body shadowing; real indoor RSSI positioning is 4–6 m error. Any absolute ATE number transfers poorly; only the *paired* deltas are defensible, and even those depend on σ_dB. State σ_dB and show a σ_dB sweep (3, 6, 9 dB) if at all possible.
6. **Wheel-scale calibration from RSSI.** With σ_d ~ metres, the scale state is weakly observable over a 280 s mission; a reviewer will ask for observability analysis or for the scale estimate's convergence plot with ground truth, and will note Nguyen 2020 / UWB-odometer literature as prior structure.
7. **n = 8 per arm**, single simulator, one map, three robots. Expected "indicative" pushback; scale n or add a second environment.
8. **ISAC framing.** The ISAC community's definition (waveform-level joint design) does not match RSSI harvesting; calling this "ISAC" without qualification invites a wireless-venue reject. Use "communication-as-sensing / opportunistic RF sensing" in the title, mention ISAC only as context.
9. **Shared map origin and known channel parameters (P_tx, PL_0, n)** — both surveyed a priori; reviewers will ask how the range model is calibrated in the field (see Yang 2025 for online range-bias calibration as a pattern).

## Reference keys suggested for PAPER.md placeholders

- `[Mostofi CompressiveCoopMapping]` → Mostofi & Gonzalez-Ruiz, MILCOM 2010 / Mostofi, TMC 2012.
- `[Mostofi SeeThrough]` → Depatla, Buckland & Mostofi, TVT 2015; Karanam & Mostofi, IPSN 2017.
- `[Mostofi IntegratedLaserWireless]` → Gonzalez-Ruiz, Ghaffarkhah & Mostofi, IEEE Sensors J. 2014.
- `[InterRanging24]` → Knowles, Dai & Gao, arXiv 2406.16679.
- `[DCLSparse24]` → Sagale, Kargar Tasooji & Parasuraman, arXiv 2412.14793.
- `[FiedlerPlanning24]` → Mikkelsen, Galeazzi & Fumagalli, arXiv 2406.18452.
- `[CommAwareTraj20]` → **confirmed:** D. Bonilla Licea, M. Bonilla, M. Ghogho, S. Lasaulce & V. S. Varma, "Communication-aware energy efficient trajectory planning with limited channel knowledge," IEEE Trans. Robotics 36(2):431–442, 2020 (arXiv 2011.09206).
- `[UAVSwarmISAC25]` → Zhai, Ni, Wang, Niyato & Hossain, arXiv 2511.03283.
- `[Julier & Uhlmann 1997]` → ACC 1997, DOI 10.1109/ACC.1997.609105.
- Add: Oliveira 2014; Rubenstein 2014; Cossette 2022; Ahmed 2024; Le Ny & Chauvière 2018; Zhou & Roumeliotis 2008/2011; Carrillo-Arce 2013; Luft 2018; Chang 2021; Bullmann 2020; Borenstein & Feng 1996; Nguyen 2020; Dijkstra 2024; Clark/PropEM-L 2022; Latif & Parasuraman 2022/2024.
