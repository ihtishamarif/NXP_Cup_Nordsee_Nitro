"""Lap progress and rulebook failure checks against the track centre line.

Shared by tools/fast_sim.py and the lap_monitor node so both judge a run the
same way. No ROS imports.
"""

import math
from dataclasses import dataclass

import numpy as np

TRACK_HALF = 0.275     # m, half of the 55 cm track width
STEP = 0.005           # m between centre-line samples (generate_track.py)


@dataclass
class CarShape:
    wheel_base: float = 0.175
    wheel_separation: float = 0.150


class TrackProgress:
    """Follows the car along the centre line and reports laps and failures.

    The nearest centre-line point is searched only in a window around the last
    one, so the crossing (where two branches overlap) doesn't confuse it, and
    taking the wrong branch shows up as a failure.
    """

    def __init__(self, centerline, length_m, car: CarShape | None = None):
        self.cl = np.asarray(centerline)[:, :2]
        self.n = len(self.cl)
        self.length = length_m
        self.car = car or CarShape()
        self.reset()

    def reset(self):
        self.idx = None
        self.progress = 0          # samples driven since the start
        self.laps = 0
        self.lap_start_t = None
        self.lap_max_lateral = 0.0

    def update(self, x, y, yaw, t):
        """Returns (event, lateral) where event is None, ("lap", seconds, max_lateral_m)
        or ("fail", reason)."""
        if self.idx is None:
            self.idx = int(np.argmin(np.hypot(self.cl[:, 0] - x, self.cl[:, 1] - y)))
            self.lap_start_t = t
        win = np.arange(self.idx - 40, self.idx + 120) % self.n
        d = np.hypot(self.cl[win, 0] - x, self.cl[win, 1] - y)
        k = int(np.argmin(d))
        new = int(win[k])
        fwd = (new - self.idx) % self.n
        self.progress += fwd if fwd < self.n // 2 else fwd - self.n
        self.idx = new
        lateral = float(d[k])
        self.lap_max_lateral = max(self.lap_max_lateral, lateral)

        if lateral > 0.4:
            return ("fail", "left the track branch (wrong exit at the crossing?)"), lateral

        # rulebook: failure when more than 51 % of the wheels lose contact with the track
        c, s = math.cos(yaw), math.sin(yaw)
        L, h = self.car.wheel_base, self.car.wheel_separation / 2
        off = 0
        for wx, wy in ((0, h), (0, -h), (L, h), (L, -h)):
            gx, gy = x + c * wx - s * wy, y + s * wx + c * wy
            off += np.hypot(self.cl[win, 0] - gx, self.cl[win, 1] - gy).min() > TRACK_HALF
        if off > 2:
            return ("fail", f"{off} wheels off the track"), lateral

        if self.progress * STEP >= self.length * (self.laps + 1):
            self.laps += 1
            ev = ("lap", t - self.lap_start_t, self.lap_max_lateral)
            self.lap_start_t, self.lap_max_lateral = t, 0.0
            return ev, lateral
        return None, lateral

    @property
    def distance(self):
        return self.progress * STEP
