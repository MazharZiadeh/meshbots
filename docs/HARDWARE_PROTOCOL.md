# Hardware channel characterization — protocol

Purpose: replace the *assumed* channel parameters of the simulator
(path-loss exponent n, 1 m loss PL₀, shadowing σ_dB, per-device offsets,
fading correlation time) with **measured** ones from commodity radios, so
Campaign 4 is parameterized from data rather than guesses. This is a
characterization, not a robot experiment: two boards and a tape measure.

## Equipment

- 2–3 ESP32 dev boards (any WiFi-capable variant; ESP-NOW or plain
  802.11 beacons/probe requests both expose RSSI).
- USB power banks or laptop USB, a tape measure or laser distance meter.
- A corridor or open room ≥ 15 m, and one indoor space with a wall or
  large obstacle to put between the boards.

## Firmware (minimal)

One board transmits a short packet every 100 ms (ESP-NOW broadcast
containing a sequence number). The other board(s) log every received
packet as `t_ms, seq, rssi_dBm` over serial. Three points matter for the
numbers to be usable:

1. Fixed TX power (`esp_wifi_set_max_tx_power`) and **record it**.
2. Fixed channel; no power-saving (`esp_wifi_set_ps(WIFI_PS_NONE)`).
3. Antennas at the same height (≈ 0.3 m, robot-like), same orientation at
   every position.

## Measurement plan

### A. Path loss and shadowing (line of sight)
- Distances d ∈ {0.5, 1, 1.5, 2, 3, 4, 5, 6, 8, 10, 12, 15} m.
- At each: ≥ 300 packets (30 s), boards static, nobody moving in the
  first Fresnel zone.
- Repeat the whole sweep twice, once with TX and RX boards swapped
  (isolates per-device offset from path loss).

### B. Slow fading (temporal correlation)
- One distance (4 m), static, 10 minutes continuous. With people walking
  in the corridor normally.

### C. Obstruction
- 4 m separation with a wall / metal cabinet / a person between the
  boards; 30 s each; also the same geometry without the obstacle.

### D. Motion (optional, closest to the use case)
- Walk the RX board at a steady pace from 1 m to 12 m and back, three
  times, logging continuously; time-stamp the turn-around points.

## Analysis (one script, ~80 lines)

1. **Per-distance statistics:** mean and std of RSSI at each d.
2. **Fit** RSSI(d) = P_ref − 10 n log₁₀(d / 1 m) by least squares over the
   means → n and PL₀ (with the recorded TX power). Residual std across
   the *means* is the large-scale shadowing; std *within* a position is
   small-scale/fast fading.
3. **Device offset:** difference in fitted P_ref between the two board
   assignments in (A) — half of it is the per-device offset magnitude.
4. **Fading correlation time:** autocorrelation of the (B) series;
   τ where it drops to 1/e. Feed as `rf_fading_tau`; its std as
   `rf_fading_db`.
5. **Obstruction penalty:** mean RSSI drop in (C) per obstacle type;
   compare with the simulator's dB-per-metre penetration constants.

## Feeding it back

```bash
./scripts/run_batch.sh 8 results/c5_measured 280 \
  rf_n_exp:=<n> rf_pl0:=<PL0> rf_sigma_db:=<σ_fast> \
  rf_fading_db:=<σ_slow> rf_fading_tau:=<τ> rf_offset_db:=<offset>
```

The robots keep assuming `rf_model.py`'s constants; the difference between
those and the fitted values is exactly the model mismatch a field
deployment would face. Report the fitted numbers in `docs/RESULTS.md`
next to the Campaign 4 sweep, and the range σ_d they imply at 3 and 10 m
(σ_d ≈ d · ln10 · σ_dB / 10n) so a reader can see at a glance how much
worse than the idealized channel the measured one is.

## What this does and does not buy

It turns "idealized channel" into "channel parameterized from measurement"
— the cheapest credibility upgrade available — and gives real per-device
offsets and fading numbers that the offset/mismatch arms currently
guess. It does **not** validate the robot experiment itself; that needs
radios on moving platforms with ground truth, which is a different project.
