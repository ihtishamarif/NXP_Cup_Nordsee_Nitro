#!/usr/bin/env python3
"""Fast closed-loop test of the line follower without ROS or Gazebo.

Renders the car's camera view straight from the generated track texture with the
same pinhole model as the Gazebo camera, runs nxp_cup_control.lane_tracker on it,
and drives a kinematic bicycle model (steering servo lag, acceleration limit,
one frame of latency). Checks the rulebook failure conditions for a lap:
more than half the wheels off the track, or leaving a crossing on the wrong branch.

    python3 tools/fast_sim.py                      # one lap, summary
    python3 tools/fast_sim.py --laps 3 --video lap.mp4
    python3 tools/fast_sim.py --v-max 2.0 --kp 1.4  # try other gains
    python3 tools/fast_sim.py --y-off 0.12 --yaw-off -0.3   # crooked start

Run generate_track.py in nxp_cup_gazebo first if you changed the layout.
"""

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from nxp_cup_control.lane_tracker import (CameraGeometry, ControlParams,  # noqa: E402
                                          LineFollower, TrackerParams, draw_debug)

GAZEBO_PKG = HERE.parent.parent / "nxp_cup_gazebo"
SKY_BGR = (217, 204, 191)
TRACK_HALF = 0.275


class Camera:
    """Pinhole camera on the car; precomputes where each pixel hits the ground."""

    def __init__(self, width=320, height=240, hfov=1.6, cam_h=0.25, cam_x=0.10, pitch=0.75, ss=2):
        self.w, self.h, self.ss = width, height, ss
        W, H = width * ss, height * ss
        fx = (W / 2) / math.tan(hfov / 2)
        u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
        # Gazebo camera link frame: x forward, y left, z up
        x = np.ones_like(u)
        y = -(u - W / 2) / fx
        z = -(v - H / 2) / fx
        c, s = math.cos(pitch), math.sin(pitch)
        xv, zv = c * x + s * z, -s * x + c * z
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(zv < -1e-6, cam_h / -zv, np.inf)
        self.ground = (t * xv < 10.0) & np.isfinite(t)
        self.gx = np.where(self.ground, t * xv + cam_x, 0.0)   # metres ahead of the rear axle
        self.gy = np.where(self.ground, t * y, 0.0)            # metres to the left

    def render(self, tex, extent, ppm, pose, rng=None):
        xmin, ymax = extent["xmin"], extent["ymax"]
        px, py, th = pose
        c, s = math.cos(th), math.sin(th)
        X = px + c * self.gx - s * self.gy
        Y = py + s * self.gx + c * self.gy
        mapx = ((X - xmin) * ppm).astype(np.float32)
        mapy = ((ymax - Y) * ppm).astype(np.float32)
        img = cv2.remap(tex, mapx, mapy, cv2.INTER_LINEAR, borderValue=(95, 95, 95))
        img[~self.ground] = SKY_BGR
        img = cv2.resize(img, (self.w, self.h), interpolation=cv2.INTER_AREA)
        if rng is not None:   # same noise level as the Gazebo sensor (stddev 0.007)
            img = np.clip(img + rng.normal(0, 0.007 * 255, img.shape), 0, 255).astype(np.uint8)
        return img


def load_track():
    import yaml
    info = yaml.safe_load((GAZEBO_PKG / "config" / "track_info.yaml").read_text())
    model = GAZEBO_PKG / "models" / "nxp_cup_track"
    tex = cv2.imread(str(model / "meshes" / "track.png"))
    cl = np.loadtxt(model / "centerline.csv", delimiter=",", skiprows=1)
    return info, tex, cl


def run(args):
    info, tex, cl = load_track()
    ppm, ext = info["texture_px_per_m"], info["extent"]
    cam = Camera(hfov=args.hfov, cam_h=args.cam_h, pitch=args.pitch)
    cp = ControlParams(kp=args.kp, kd=args.kd, kf=args.kf, v_max=args.v_max, v_min=args.v_min)
    follower = LineFollower(TrackerParams(), cp,
                            CameraGeometry(height=args.cam_h, pitch=args.pitch, hfov=args.hfov))
    rng = np.random.default_rng(0)

    L, half_sep = cp.wheel_base, 0.075
    x, y, th = info["spawn"]["x"], info["spawn"]["y"], info["spawn"]["yaw"]
    # optional off-centre / crooked start, offset is to the car's left
    x -= args.y_off * math.sin(th)
    y += args.y_off * math.cos(th)
    th += args.yaw_off
    v = delta = 0.0
    cmd_v = cmd_d = 0.0          # command applied (one frame late)
    pending = (0.0, 0.0)

    n_cl = len(cl)
    d0 = np.hypot(cl[:, 0] - x, cl[:, 1] - y)
    idx = int(np.argmin(d0))
    start_idx, progress = idx, 0      # progress in centre-line samples, tracked locally
    step_len = 0.005

    dt_phys, frame_dt = 0.002, 1.0 / 30
    t, next_frame = 0.0, 0.0
    max_off, sum_sq, n_samples = 0.0, 0.0, 0
    lap_times, last_lap_t = [], 0.0
    failure = None
    writer = None
    if args.video:
        writer = cv2.VideoWriter(args.video, cv2.VideoWriter_fourcc(*"mp4v"), 30, (640 + 320, 480))
    top = cv2.resize(tex, None, fx=640 / tex.shape[1], fy=640 / tex.shape[1], interpolation=cv2.INTER_AREA)

    while t < args.timeout:
        if t >= next_frame:
            next_frame += frame_dt
            cmd_v, cmd_d = pending
            img = cam.render(tex, ext, ppm, (x, y, th), rng)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            spd, st = follower.step(gray, frame_dt)
            pending = (spd, st)

            # progress along the centre line, searched locally so crossings don't confuse it
            win = np.arange(idx - 40, idx + 120) % n_cl
            dd = np.hypot(cl[win, 0] - x, cl[win, 1] - y)
            k = int(np.argmin(dd))
            new_idx = int(win[k])
            progress += (new_idx - idx) % n_cl if (new_idx - idx) % n_cl < n_cl // 2 else -((idx - new_idx) % n_cl)
            idx = new_idx
            if dd[k] > 0.4:
                failure = f"left the track branch at t={t:.2f}s (wrong exit at crossing?)"
                break

            # rulebook: failure when more than 51% of the wheels lose contact with the track
            c, s = math.cos(th), math.sin(th)
            wheels = [(0, half_sep), (0, -half_sep), (L, half_sep), (L, -half_sep)]
            off = 0
            for wx, wy in wheels:
                gx, gy = x + c * wx - s * wy, y + s * wx + c * wy
                dw = np.hypot(cl[win, 0] - gx, cl[win, 1] - gy).min()
                off += dw > TRACK_HALF
            lateral = dd[k]
            max_off = max(max_off, lateral)
            sum_sq += lateral ** 2
            n_samples += 1
            if off > 2:
                failure = f"{off} wheels off the track at t={t:.2f}s, x={x:.2f} y={y:.2f}"
                break
            if progress * step_len >= info["length_m"] * (len(lap_times) + 1):
                lap_times.append(t - last_lap_t)
                last_lap_t = t
                if len(lap_times) >= args.laps:
                    break

            if writer is not None:
                dbg = cv2.resize(draw_debug(img, follower), (640, 480), interpolation=cv2.INTER_NEAREST)
                mini = top.copy()
                sc = 640 / tex.shape[1]
                cv2.circle(mini, (int((x - ext["xmin"]) * ppm * sc), int((ext["ymax"] - y) * ppm * sc)),
                           5, (0, 0, 255), -1)
                pane = np.full((480, 320, 3), 40, np.uint8)
                mh = min(480, mini.shape[0] * 320 // mini.shape[1])
                pane[:mh] = cv2.resize(mini, (320, mh))
                writer.write(np.hstack([dbg, pane]))

        # plant: servo-like steering (matches steer_p_gain in the URDF), accel limit
        delta += np.clip(15.0 * (cmd_d - delta), -10.0, 10.0) * dt_phys
        delta = float(np.clip(delta, -cp.steering_limit, cp.steering_limit))
        v += float(np.clip(cmd_v - v, -4.0 * dt_phys, 4.0 * dt_phys))
        x += v * math.cos(th) * dt_phys
        y += v * math.sin(th) * dt_phys
        th += v * math.tan(delta) / L * dt_phys
        t += dt_phys
    else:
        failure = failure or f"timeout after {args.timeout}s ({progress*step_len:.1f} m driven)"

    if writer is not None:
        writer.release()
    rms = math.sqrt(sum_sq / max(1, n_samples))
    return lap_times, max_off, rms, failure


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=120.0, help="rulebook: 120 s per attempt")
    ap.add_argument("--video", help="write an mp4 of camera view + debug overlay")
    ap.add_argument("--y-off", type=float, default=0.0, help="start offset to the left, m")
    ap.add_argument("--yaw-off", type=float, default=0.0, help="start heading error, rad")
    ap.add_argument("--hfov", type=float, default=1.6)
    ap.add_argument("--cam-h", type=float, default=0.25)
    ap.add_argument("--pitch", type=float, default=0.75)
    d = ControlParams()
    ap.add_argument("--kp", type=float, default=d.kp)
    ap.add_argument("--kd", type=float, default=d.kd)
    ap.add_argument("--kf", type=float, default=d.kf)
    ap.add_argument("--v-max", type=float, default=d.v_max)
    ap.add_argument("--v-min", type=float, default=d.v_min)
    args = ap.parse_args()

    laps, max_off, rms, failure = run(args)
    for i, lt in enumerate(laps, 1):
        print(f"lap {i}: {lt:.2f} s")
    print(f"centre-line deviation: max {max_off*100:.1f} cm, rms {rms*100:.1f} cm")
    if failure:
        print("FAILED:", failure)
        sys.exit(1)


if __name__ == "__main__":
    main()
