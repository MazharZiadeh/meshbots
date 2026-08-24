"""Shared RF propagation model — the heart of the ISAC layer.

Log-distance path loss with per-obstacle penetration attenuation:

    RSSI(d) = P_TX - (PL0 + 10 n log10(d/1m)) - sum_k alpha_k * chord_k + N(0, sigma)

where chord_k is the length of the link segment passing through obstacle k
(computed against the same geometry as worlds/arena.sdf). Packet delivery is
Bernoulli with a logistic PDR curve around receiver sensitivity, so links
degrade gracefully instead of cliff-dropping.

The same model provides the two ISAC inversions used on-robot:
* rssi_to_range: RSSI -> range estimate + standard deviation
  (cooperative-localization factor),
* excess loss vs. free-space prediction -> evidence of an obstruction on the
  inter-robot ray (RF-shadow mapping).
"""
import math

P_TX = 20.0        # dBm
PL0 = 55.0         # dB at 1 m
N_EXP = 2.5        # path-loss exponent
SIGMA_DB = 2.0     # shadowing noise per measurement
SENS = -62.0       # receiver sensitivity midpoint (PDR = 0.5)
PDR_SCALE = 2.0    # logistic softness in dB

# Obstacles, mirroring worlds/arena.sdf. Attenuation in dB per metre of
# penetration. Rovers operate inside the outer walls, so those never
# intersect a link and are omitted.
BOXES = [   # (cx, cy, size_x, size_y, yaw, alpha)
    (-4.0,  2.0, 2.0, 2.0,  0.4, 5.0),   # crate_1 (wood)
    ( 5.0, -4.0, 2.5, 1.5, -0.3, 5.0),   # crate_2
    ( 2.0,  8.0, 1.8, 1.8,  0.9, 5.0),   # crate_3
]
CIRCLES = [  # (cx, cy, r, alpha)
    (-7.0, -6.0, 1.0, 9.0),              # silo_1 (concrete)
    ( 9.0,  5.0, 1.2, 9.0),              # silo_2
    (-2.0, -10.0, 0.8, 9.0),             # silo_3
]


def _seg_circle_chord(ax, ay, bx, by, cx, cy, r):
    dx, dy = bx - ax, by - ay
    seg_len2 = dx * dx + dy * dy
    if seg_len2 < 1e-12:
        return 0.0
    fx, fy = ax - cx, ay - cy
    a = seg_len2
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r
    disc = b * b - 4.0 * a * c
    if disc <= 0.0:
        return 0.0
    sq = math.sqrt(disc)
    t0 = max(0.0, (-b - sq) / (2.0 * a))
    t1 = min(1.0, (-b + sq) / (2.0 * a))
    if t1 <= t0:
        return 0.0
    return (t1 - t0) * math.sqrt(seg_len2)


def _seg_obb_chord(ax, ay, bx, by, cx, cy, sx, sy, yaw):
    # Transform segment into the box frame, then slab-clip.
    c, s = math.cos(-yaw), math.sin(-yaw)

    def to_box(px, py):
        px, py = px - cx, py - cy
        return c * px - s * py, s * px + c * py

    ax, ay = to_box(ax, ay)
    bx, by = to_box(bx, by)
    dx, dy = bx - ax, by - ay
    t0, t1 = 0.0, 1.0
    for p, d, half in ((ax, dx, sx / 2.0), (ay, dy, sy / 2.0)):
        if abs(d) < 1e-12:
            if abs(p) > half:
                return 0.0
            continue
        ta = (-half - p) / d
        tb = (half - p) / d
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 >= t1:
            return 0.0
    return (t1 - t0) * math.hypot(dx, dy)


def obstruction_loss(ax, ay, bx, by):
    """Total penetration attenuation (dB) along the link segment."""
    loss = 0.0
    for cx, cy, sx, sy, yaw, alpha in BOXES:
        loss += alpha * _seg_obb_chord(ax, ay, bx, by, cx, cy, sx, sy, yaw)
    for cx, cy, r, alpha in CIRCLES:
        loss += alpha * _seg_circle_chord(ax, ay, bx, by, cx, cy, r)
    return loss


def free_space_rssi(d):
    d = max(d, 0.3)
    return P_TX - (PL0 + 10.0 * N_EXP * math.log10(d))


def link_rssi(ax, ay, bx, by, noise=0.0):
    """Mean RSSI for a link between two world points (+ optional noise term)."""
    d = math.hypot(bx - ax, by - ay)
    return free_space_rssi(d) - obstruction_loss(ax, ay, bx, by) + noise


def pdr(rssi):
    """Packet delivery ratio for a given RSSI."""
    return 1.0 / (1.0 + math.exp((SENS - rssi) / PDR_SCALE))


def rssi_to_range(rssi):
    """Invert path loss: RSSI -> (range estimate, 1-sigma std dev).

    Assumes free space; obstructed links read long, which is why consumers
    gate factors against their occupancy map before trusting them.
    """
    d = 10.0 ** ((P_TX - PL0 - rssi) / (10.0 * N_EXP))
    sigma = d * math.log(10.0) * SIGMA_DB / (10.0 * N_EXP)
    return d, max(sigma, 0.15)
