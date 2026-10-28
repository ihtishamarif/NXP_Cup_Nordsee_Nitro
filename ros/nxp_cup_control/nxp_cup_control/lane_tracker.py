"""Camera line follower core for the NXP Cup. No ROS imports on purpose.

The method is deliberately MCU-friendly so it can be ported to C later:
  * average a few image rows into a 1-D brightness profile (like a line-scan
    camera), at a "near" row and a "far" row
  * in each profile find the inner edges of the two black track lines,
    searching only in a small window around where they were last frame
  * if one line is missing, infer it from the learned lane width; if both are
    missing (crossing, glare) hold the last command for a short time
  * steer with PD on the near-row offset plus a feed-forward term from the
    far-row offset; slow down when the far row says a curve is coming

Everything is integer-friendly: no floats are needed per pixel.
"""

from dataclasses import dataclass, field
import math

import numpy as np

TRACK_WIDTH_M = 0.55 - 2 * 0.02   # distance between the inner edges of the lines


@dataclass
class CameraGeometry:
    """Mounting of the camera; used to predict the lane width in pixels per row."""
    height: float = 0.25   # lens above ground, m
    pitch: float = 0.75    # tilt down from horizontal, rad
    hfov: float = 1.6      # horizontal field of view, rad

    def lane_px(self, row_frac, w, h):
        """Expected distance between the lines' inner edges at an image row, in pixels."""
        fx = (w / 2) / math.tan(self.hfov / 2)
        v = row_frac * (h - 1) + 0.5
        depth_inv = (math.sin(self.pitch) + math.cos(self.pitch) * (v - h / 2) / fx) / self.height
        if depth_inv <= 0:
            raise ValueError(f"scan row {row_frac} is above the horizon")
        return TRACK_WIDTH_M * fx * depth_inv


@dataclass
class TrackerParams:
    near_row: float = 0.50       # scan row, fraction of image height from the top
    far_row: float = 0.35
    band: int = 3                # rows averaged per scan line
    window: float = 0.10         # search window around the last edge, fraction of width
    min_contrast: int = 50       # max-min brightness needed to trust a row
    width_tolerance: float = 0.20  # reject a pair if lane width is off by more than this
    lost_frames: int = 8         # frames without any edge before re-initialising


@dataclass
class ScanResult:
    left: float | None = None     # pixel column of the left line's inner edge
    right: float | None = None
    center: float | None = None
    width: float | None = None    # learned lane width in pixels at this row
    measured: int = 0             # how many edges were really seen (0, 1, 2)


@dataclass
class ScanLine:
    """Tracks the two lane edges along one image row."""
    params: TrackerParams
    row_frac: float
    left: float | None = None
    right: float | None = None
    width: float | None = None
    nominal: float | None = None  # lane width predicted from the camera geometry
    missing: int = 0
    last: ScanResult = field(default_factory=ScanResult)

    @staticmethod
    def _edge(profile, thr, start, stop):
        """First white->dark transition walking from start towards stop."""
        step = 1 if stop > start else -1
        prev_white = profile[start] >= thr
        for i in range(start + step, stop + step, step):
            dark = profile[i] < thr
            if dark and prev_white:
                return i - 0.5 * step
            prev_white = not dark
        return None

    def update(self, gray):
        h, w = gray.shape
        r = int(self.row_frac * (h - 1))
        b = self.params.band
        profile = gray[max(0, r - b // 2): r + b // 2 + 1].astype(np.int32).mean(axis=0)

        lo, hi = profile.min(), profile.max()
        res = ScanResult(width=self.width)
        if hi - lo < self.params.min_contrast:
            return self._miss(res)
        thr = lo + 0.5 * (hi - lo)

        if self.left is None or self.right is None:
            return self._init(profile, thr, w, res)

        win = max(2, int(self.params.window * w))
        clamp = lambda x: int(min(w - 1, max(0, x)))
        # walk outward: for the left edge from inside (right) to outside (left)
        L = self._edge(profile, thr, clamp(self.left + win), clamp(self.left - win))
        R = self._edge(profile, thr, clamp(self.right - win), clamp(self.right + win))

        if L is not None and R is not None:
            if abs((R - L) - self.width) > self.params.width_tolerance * self.width:
                L = R = None              # e.g. start line dashes, reflections
            else:
                self.width += 0.05 * ((R - L) - self.width)
                if self.nominal:   # never drift far from what the geometry says
                    lim = self.params.width_tolerance * self.nominal
                    self.width = min(self.nominal + lim, max(self.nominal - lim, self.width))
        measured = (L is not None) + (R is not None)
        if measured == 0:
            return self._miss(res)
        if L is None:
            L = R - self.width
        if R is None:
            R = L + self.width
        self.left, self.right, self.missing = L, R, 0
        res.left, res.right, res.measured = L, R, measured
        res.center, res.width = 0.5 * (L + R), self.width
        self.last = res
        return res

    def _init(self, profile, thr, w, res):
        """(Re)initialise: walk outward from a few seeds inside the lane.

        Accepts a pair whose width is plausible, or a single edge when the other
        side runs to the image border without one (line out of view), as long
        as the lane width is known from the geometry or earlier frames.
        """
        if self.width is None:
            self.width = self.nominal
        tol = self.params.width_tolerance
        for s in (w // 2, w // 2 - w // 8, w // 2 + w // 8, w // 4, 3 * w // 4):
            if profile[s] < thr:
                continue
            L = self._edge(profile, thr, s, 0)
            R = self._edge(profile, thr, s, w - 1)
            if L is not None and R is not None:
                if R - L < 0.2 * w:
                    continue                       # between start-line dashes
                if self.width is not None and abs((R - L) - self.width) > tol * self.width:
                    continue
                measured = 2
            elif self.width is None or (L is None and R is None):
                continue
            elif L is None:
                L, measured = R - self.width, 1
            else:
                R, measured = L + self.width, 1
            if self.width is None:
                self.width = R - L
            self.left, self.right, self.missing = L, R, 0
            res.left, res.right, res.measured = L, R, measured
            res.center, res.width = 0.5 * (L + R), self.width
            self.last = res
            return res
        return self._miss(res)

    def _miss(self, res):
        self.missing += 1
        if self.missing > self.params.lost_frames:
            self.left = self.right = None
        res.measured = 0
        self.last = res
        return res


@dataclass
class ControlParams:
    wheel_base: float = 0.175     # m
    steering_limit: float = 0.45  # rad
    kp: float = 1.2               # rad of steering per metre of near-row offset
    kd: float = 0.05              # rad per (m/s)
    kf: float = 0.9               # feed-forward from far-row offset, rad per metre
    v_max: float = 1.5            # m/s on straights
    v_min: float = 0.8            # m/s in the tightest curves
    slow_offset: float = 0.15     # far-row offset (m) at which we are down to v_min
    hold_time: float = 0.6        # s to keep going on the last command with no lines
    v_hold: float = 0.8           # m/s while holding
    accel_limit: float = 3.5      # m/s^2 ramp on the speed output; keep it below the
                                  # drive's own limit (URDF max_acceleration = 4) so the
                                  # speed we compute yaw rate from is the speed it really runs


class LineFollower:
    """Image in, (speed m/s, steering rad) out."""

    def __init__(self, tracker: TrackerParams | None = None, control: ControlParams | None = None,
                 camera: CameraGeometry | None = None):
        self.tp = tracker or TrackerParams()
        self.cp = control or ControlParams()
        self.cam = camera or CameraGeometry()
        self.near = ScanLine(self.tp, self.tp.near_row)
        self.far = ScanLine(self.tp, self.tp.far_row)
        self.prev_err = None
        self.steer = 0.0
        self.speed = 0.0
        self.lost_for = 0.0

    @staticmethod
    def _offset_m(res, w):
        """Lane centre offset from the image centre, in metres at that row (right = +)."""
        return (res.center - 0.5 * (w - 1)) / res.width * TRACK_WIDTH_M

    def step(self, gray, dt):
        h, w = gray.shape
        if self.near.nominal is None:
            self.near.nominal = self.cam.lane_px(self.tp.near_row, w, h)
            self.far.nominal = self.cam.lane_px(self.tp.far_row, w, h)
        n = self.near.update(gray)
        f = self.far.update(gray)
        cp = self.cp

        if n.measured == 0 and f.measured == 0:
            self.lost_for += dt
            target = 0.0 if self.lost_for > cp.hold_time else min(self.speed, cp.v_hold)
            self.prev_err = None
            return self._ramp(target, dt), self.steer
        self.lost_for = 0.0

        e_near = self._offset_m(n, w) if n.measured else None
        e_far = self._offset_m(f, w) if f.measured else None
        if e_near is None:
            e_near = 0.5 * e_far
        if e_far is None:
            e_far = e_near

        d = 0.0 if self.prev_err is None or dt <= 0 else (e_near - self.prev_err) / dt
        self.prev_err = e_near
        steer = -(cp.kp * e_near + cp.kd * d + cp.kf * e_far)
        self.steer = max(-cp.steering_limit, min(cp.steering_limit, steer))

        slow = min(1.0, abs(e_far) / cp.slow_offset)
        return self._ramp(cp.v_max - (cp.v_max - cp.v_min) * slow, dt), self.steer

    def _ramp(self, target, dt):
        dv = self.cp.accel_limit * max(dt, 0.0)
        self.speed += max(-dv, min(dv, target - self.speed))
        return self.speed

    def yaw_rate(self):
        """Ackermann: convert the steering angle to angular.z for cmd_vel.

        Gazebo's AckermannSteering turns (linear.x, angular.z) back into a steering
        angle with R = v / angular.z, using its acceleration-limited copy of v.
        self.speed is already ramped below that limit, so both agree and the car
        steers exactly self.steer.
        """
        return self.speed * math.tan(self.steer) / self.cp.wheel_base


def draw_debug(bgr, follower: LineFollower):
    """Overlay scan rows and detected edges on a BGR image (needs OpenCV)."""
    import cv2
    out = bgr.copy()
    h, w = out.shape[:2]
    for sl, col in ((follower.near, (0, 200, 0)), (follower.far, (255, 120, 0))):
        r = int(sl.row_frac * (h - 1))
        cv2.line(out, (0, r), (w - 1, r), col, 1)
        res = sl.last
        for x, c in ((res.left, (0, 0, 255)), (res.right, (0, 0, 255)), (res.center, col)):
            if x is not None:
                cv2.circle(out, (int(round(x)), r), 4, c, -1)
    cv2.putText(out, f"v={follower.speed:.2f} d={math.degrees(follower.steer):+.0f}",
                (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
    return out
