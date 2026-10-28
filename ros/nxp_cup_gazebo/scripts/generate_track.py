#!/usr/bin/env python3
"""Generate an NXP Cup track for Gazebo from a list of track segments.

Track spec (NXP Cup rulebook 2026/27, "The Race Track"):
  * track width 55 cm
  * matte white surface, continuous 2 cm black line on each edge
  * crossings at 45 or 90 degrees are allowed

A layout is a list of segments driven like a turtle along the centre line:
  ("S", length_m)              straight
  ("L", radius_m, angle_deg)   left arc
  ("R", radius_m, angle_deg)   right arc
  ("CHICANE", radius_m, angle_deg)  L a, R a, R a, L a  (returns to the same line);
                                    a negative angle mirrors it (R first)

Outputs (all relative to the nxp_cup_gazebo package):
  models/nxp_cup_track/        Gazebo model: textured OBJ quad + flat collision box
  models/nxp_cup_track/centerline.csv   x, y, heading of the centre line (5 mm steps)
  config/track_info.yaml       layout name, size, recommended spawn pose

Usage:
  python3 generate_track.py                 # default layout (figure8)
  python3 generate_track.py --layout oval
  python3 generate_track.py --list
"""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

TRACK_WIDTH = 0.55   # m, rulebook
LINE_WIDTH = 0.02    # m, rulebook
MARGIN = 0.40        # m of floor around the track inside the texture
PPM = 500            # texture pixels per metre (2 mm per pixel)
STEP = 0.005         # centre-line sampling step in metres

FLOOR_BGR = (95, 95, 95)  # grey hall floor around the track

# Each layout: start pose of the centre line, the segments, and where the
# start/finish line sits (distance along the path, in metres).
LAYOUTS = {
    # Figure-eight with a 90 degree crossing at the origin and two chicanes.
    # Fits in 5.75 m x 3.95 m, inside the 7 m x 4 m example from the rulebook.
    "figure8": {
        "start": (0.0, 0.0, 0.0),
        "segments": [
            ("S", 0.4), ("CHICANE", 0.6, 30), ("S", 0.4),   # east, 2.0 m
            ("L", 0.6, 90), ("S", 0.5), ("L", 0.6, 90),
            ("S", 1.4),
            ("L", 0.6, 90),
            ("S", 2.2),                                     # through the crossing
            ("R", 0.6, 90),
            ("S", 0.1), ("CHICANE", 0.6, -30), ("S", 0.1),  # west, 1.4 m
            ("R", 0.6, 90), ("S", 0.5), ("R", 0.6, 90),
            ("S", 2.0),                                     # back into the crossing
        ],
        "start_line_s": None,     # filled below: 0.8 m before the end
        "start_line_back": 0.8,
    },
    # Simple oval for first controller tuning.
    "oval": {
        "start": (-1.25, -0.9, 0.0),
        "segments": [
            ("S", 2.5), ("L", 0.9, 180), ("S", 2.5), ("L", 0.9, 180),
        ],
        "start_line_s": 1.0,
    },
}


def expand(segments):
    out = []
    for seg in segments:
        if seg[0] == "CHICANE":
            _, r, a = seg
            first, second = ("L", "R") if a > 0 else ("R", "L")
            a = abs(a)
            out += [(first, r, a), (second, r, a), (second, r, a), (first, r, a)]
        else:
            out.append(seg)
    return out


def sample_centerline(start, segments, step=STEP):
    """Return an (N, 3) array of x, y, heading along the centre line."""
    x, y, th = start
    pts = [(x, y, th)]
    for seg in expand(segments):
        kind = seg[0]
        if kind == "S":
            length = seg[1]
            n = max(1, int(round(length / step)))
            ds = length / n
            for _ in range(n):
                x += ds * math.cos(th)
                y += ds * math.sin(th)
                pts.append((x, y, th))
        elif kind in ("L", "R"):
            r, ang = seg[1], math.radians(seg[2])
            sign = 1.0 if kind == "L" else -1.0
            # centre of the arc, perpendicular to the heading
            cx = x - sign * r * math.sin(th)
            cy = y + sign * r * math.cos(th)
            phi0 = math.atan2(y - cy, x - cx)
            n = max(2, int(round(r * ang / step)))
            for i in range(1, n + 1):
                phi = phi0 + sign * ang * i / n
                th_i = th + sign * ang * i / n
                pts.append((cx + r * math.cos(phi), cy + r * math.sin(phi), th_i))
            x, y, th = pts[-1]
        else:
            raise ValueError(f"unknown segment {seg}")
    arr = np.array(pts)
    arr[:, 2] = np.arctan2(np.sin(arr[:, 2]), np.cos(arr[:, 2]))
    return arr


def check_closed(cl):
    dpos = math.hypot(cl[-1, 0] - cl[0, 0], cl[-1, 1] - cl[0, 1])
    dth = abs(math.atan2(math.sin(cl[-1, 2] - cl[0, 2]), math.cos(cl[-1, 2] - cl[0, 2])))
    if dpos > 0.01 or dth > math.radians(1):
        raise SystemExit(f"layout does not close: gap {dpos*100:.1f} cm, "
                         f"heading error {math.degrees(dth):.1f} deg")


def render(cl, start_line_s):
    xmin, ymin = cl[:, 0].min() - TRACK_WIDTH / 2 - MARGIN, cl[:, 1].min() - TRACK_WIDTH / 2 - MARGIN
    xmax, ymax = cl[:, 0].max() + TRACK_WIDTH / 2 + MARGIN, cl[:, 1].max() + TRACK_WIDTH / 2 + MARGIN
    w_px = int(math.ceil((xmax - xmin) * PPM))
    h_px = int(math.ceil((ymax - ymin) * PPM))
    # snap the extents to whole pixels so world<->pixel mapping is exact
    xmax, ymin = xmin + w_px / PPM, ymax - h_px / PPM

    def to_px(xy):
        # image row 0 is +y max (top); column 0 is x min (left)
        col = (xy[:, 0] - xmin) * PPM
        row = (ymax - xy[:, 1]) * PPM
        return np.stack([col, row], axis=1)

    img = np.full((h_px, w_px, 3), FLOOR_BGR, np.uint8)
    poly = np.round(to_px(cl[:, :2]) * 16).astype(np.int32).reshape(-1, 1, 2)
    # Black band the full track width, then white over it leaving 2 cm edges.
    # Drawing all black first and all white second makes crossings come out
    # like the real crossing tile: white square, edge lines interrupted.
    cv2.polylines(img, [poly], True, (20, 20, 20), int(round(TRACK_WIDTH * PPM)),
                  cv2.LINE_AA, shift=4)
    cv2.polylines(img, [poly], True, (245, 245, 245),
                  int(round((TRACK_WIDTH - 2 * LINE_WIDTH) * PPM)), cv2.LINE_AA, shift=4)

    if start_line_s is not None:
        # dashed start/finish line across the track, 2 cm wide, clear of the edge lines
        i = int(round(start_line_s / STEP))
        x, y, th = cl[i]
        nx, ny = -math.sin(th), math.cos(th)
        half = TRACK_WIDTH / 2 - 0.08
        n_dash = 5
        for k in range(n_dash):
            a = -half + 2 * half * (2 * k) / (2 * n_dash - 1)
            b = -half + 2 * half * (2 * k + 1) / (2 * n_dash - 1)
            seg = np.array([[x + a * nx, y + a * ny], [x + b * nx, y + b * ny]])
            p = np.round(to_px(seg) * 16).astype(np.int32)
            cv2.line(img, tuple(p[0]), tuple(p[1]), (20, 20, 20),
                     int(round(LINE_WIDTH * PPM)), cv2.LINE_AA, shift=4)
    return img, (xmin, xmax, ymin, ymax)


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>nxp_cup_track</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>Nordsee Nitro</name></author>
  <description>NXP Cup track ({layout}), generated by scripts/generate_track.py</description>
</model>
"""

MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="nxp_cup_track">
    <static>true</static>
    <link name="track">
      <collision name="collision">
        <pose>{cx:.4f} {cy:.4f} -0.005 0 0 0</pose>
        <geometry><box><size>{w:.4f} {h:.4f} 0.01</size></box></geometry>
        <surface>
          <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh><uri>model://nxp_cup_track/meshes/track.obj</uri></mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""

OBJ = """# NXP Cup track surface, generated by generate_track.py
mtllib track.mtl
v {x0:.4f} {y0:.4f} 0.0
v {x1:.4f} {y0:.4f} 0.0
v {x1:.4f} {y1:.4f} 0.0
v {x0:.4f} {y1:.4f} 0.0
vt 0 0
vt 1 0
vt 1 1
vt 0 1
vn 0 0 1
usemtl track
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
"""

MTL = """newmtl track
Ka 1.0 1.0 1.0
Kd 1.0 1.0 1.0
Ks 0.0 0.0 0.0
Ns 1.0
d 1.0
illum 1
map_Kd track.png
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--layout", default="figure8", choices=sorted(LAYOUTS))
    ap.add_argument("--list", action="store_true", help="list layouts and exit")
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parent.parent,
                    help="nxp_cup_gazebo package directory")
    args = ap.parse_args()
    if args.list:
        for name in sorted(LAYOUTS):
            print(name)
        return

    lay = LAYOUTS[args.layout]
    cl = sample_centerline(lay["start"], lay["segments"])
    check_closed(cl)
    length = (len(cl) - 1) * STEP
    s_line = lay.get("start_line_s")
    if s_line is None:
        s_line = length - lay["start_line_back"]

    img, (xmin, xmax, ymin, ymax) = render(cl, s_line)

    model_dir = args.out / "models" / "nxp_cup_track"
    (model_dir / "meshes").mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(model_dir / "meshes" / "track.png"), img)
    (model_dir / "meshes" / "track.obj").write_text(OBJ.format(x0=xmin, x1=xmax, y0=ymin, y1=ymax))
    (model_dir / "meshes" / "track.mtl").write_text(MTL)
    (model_dir / "model.config").write_text(MODEL_CONFIG.format(layout=args.layout))
    (model_dir / "model.sdf").write_text(MODEL_SDF.format(
        cx=(xmin + xmax) / 2, cy=(ymin + ymax) / 2, w=xmax - xmin, h=ymax - ymin))
    np.savetxt(model_dir / "centerline.csv", cl, delimiter=",", fmt="%.4f",
               header="x,y,heading", comments="")

    # Spawn the car on the centre line, front bumper ~25 cm behind the start line
    # (rulebook: at most 30 cm in front of the start line).
    i_spawn = int(round((s_line - 0.25 - 0.15) / STEP)) % len(cl)
    sx, sy, sth = cl[i_spawn]
    lx, ly, _ = cl[int(round(s_line / STEP))]
    # "End of race" stop obstacle (20 cm white cube) on the centre line, a short
    # way past the start/finish line, like the rulebook example layout.
    ox, oy, oth = cl[int(round((s_line + 0.45) / STEP)) % len(cl)]
    cfg = args.out / "config"
    cfg.mkdir(exist_ok=True)
    (cfg / "track_info.yaml").write_text(
        "# generated by scripts/generate_track.py - do not edit by hand\n"
        f"layout: {args.layout}\n"
        f"length_m: {length:.3f}\n"
        f"texture_px_per_m: {PPM}\n"
        f"extent: {{xmin: {xmin:.4f}, xmax: {xmax:.4f}, ymin: {ymin:.4f}, ymax: {ymax:.4f}}}\n"
        f"start_line: {{x: {lx:.4f}, y: {ly:.4f}}}\n"
        f"spawn: {{x: {sx:.4f}, y: {sy:.4f}, yaw: {sth:.4f}}}\n"
        f"obstacle: {{x: {ox:.4f}, y: {oy:.4f}, yaw: {oth:.4f}}}\n")
    print(f"{args.layout}: {length:.2f} m lap, track area {xmax-xmin:.2f} x {ymax-ymin:.2f} m "
          f"(incl. {MARGIN} m margin), texture {img.shape[1]}x{img.shape[0]} px")


if __name__ == "__main__":
    main()
