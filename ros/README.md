# Simulation (ROS 2 Lyrical + Gazebo Jetty)

Three packages:

| Package | What it holds |
|---|---|
| `nxp_cup_description` | Car model (xacro): Ackermann steering, forward camera, IMU |
| `nxp_cup_gazebo` | Track generator, world, "end of race" cube, ROS-Gazebo bridge, `sim.launch.py` |
| `nxp_cup_control` | Line follower: ROS-free core (`lane_tracker.py`), ROS 2 node, lap monitor, fast test bench |

The track follows the rulebook: 55 cm wide, matte white, 2 cm black line on each edge,
90° crossing. The default layout is a figure eight (5.75 m x 3.95 m, 15.8 m lap) with two
chicanes and a dashed start/finish line.

## Build

The repo is the `src` folder of a colcon workspace:

```bash
mkdir -p ~/nxp_ws/src && cd ~/nxp_ws/src
git clone git@github.com:ihtishamarif/NXP_Cup_Nordsee_Nitro.git
cd ~/nxp_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# Gazebo with the car on the start line
ros2 launch nxp_cup_gazebo sim.launch.py

# ... and let the line follower drive
ros2 launch nxp_cup_gazebo sim.launch.py follower:=true

# with the 20 cm stop cube just past the start line
ros2 launch nxp_cup_gazebo sim.launch.py follower:=true obstacle:=true

# no Gazebo window (the camera still renders)
ros2 launch nxp_cup_gazebo sim.launch.py follower:=true gui:=false

# TSL1401 line-scan camera instead of the 320x240 camera
ros2 launch nxp_cup_gazebo sim.launch.py camera:=linescan follower:=true
```

The lap monitor runs by default. It prints `LAP n: xx.xx s` in the launch terminal, and
when the car goes off the track (rulebook: more than half the wheels off, or wrong exit at
the crossing) it saves a report to `~/nxp_cup_runs/<date-time>/`:

- `failure.mp4`: the last 4 s of the follower's debug view, with commanded vs actual
  steering angle on every frame
- `log.csv`: the last 10 s of true pose, distance from the centre line, speed and steering
- `summary.txt`: where and why

Reset the world in Gazebo to start a new attempt; the monitor re-arms itself.

The line follower prints a stats line every 5 s:

```
camera 29.8 fps (sim time), image age avg 45 ms max 70 ms, processing 1.2 ms/frame, real-time factor 0.93
```

The controller wants about 30 fps and less than 0.1 s image age. Below that it starts to
weave (`fast_sim.py --fps 15 --latency 0.2` shows the effect). The default top speed of
1.0 m/s holds even then; raise `v_max` once the stats look healthy. A slow camera in
Gazebo usually means it isn't rendering on the GPU (see the NVIDIA line below).

Useful while it runs:

```bash
ros2 run rqt_image_view rqt_image_view /line_follower/debug   # scan rows + detected edges
ros2 run rqt_image_view rqt_image_view /camera/image_raw
ros2 topic hz /camera/image_raw                                 # should be ~30 Hz
ros2 run teleop_twist_keyboard teleop_twist_keyboard            # drive by hand (follower:=false)
```

If Gazebo renders slowly on the laptop, run it on the NVIDIA GPU:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch nxp_cup_gazebo sim.launch.py
```

## TSL1401 line-scan camera

`camera:=linescan` swaps the car's camera for a TSL1401: one row of 128 greyscale
pixels at 100 Hz. The rules allow it because it has no processor on board. The
follower then uses `config/line_follower_linescan.yaml`, and the launch file mounts
the simulated camera from the `camera:` section of that same file.

What it needs, from the test bench:

- **The lens decides the mount.** Ready-made modules come with a lens fitted; the
  Parallax TSL1401-DB in NXP's application note AN4244 has a 7.9 mm lens, which on the
  8.1 mm sensor row gives 54° (`hfov = 2*atan(4.06 / focal_mm)`). That is the default.
  It sees roughly as wide as it is far away, so it must look far enough ahead to see the
  whole 55 cm track: at 25 cm height the original 40 cm look-ahead sees only 49 cm and
  never finds both lines.
- **Default mount for the 7.9 mm lens:** 25 cm high, tilted 0.47 rad (27°), so the line
  sits ~49 cm ahead of the lens and covers ~57 cm. 30 cm high at 0.55 rad works as well.
- **Gains:** kp 1.2, kd 0, no feed-forward (one line means no far row).
- **Results:** laps at 1.0, 1.5 and 2.0 m/s top speed (18.0 s, 13.7 s, 11.4 s) within
  6-8 cm of the centre line; also at 50 Hz with extra delay and from crooked starts.
  A wider 6 mm lens (68°) can look closer (~40 cm) with kp 2.0.
- **Lens vignetting:** cheap lenses darken the image edges. Up to ~40% loss needs nothing;
  beyond that set `tracker.vignette_comp` to the loss you measure (point the camera at
  plain white and compare edge vs centre). A rough value is enough.

`/line_follower/debug` shows the brightness profile: the stretched pixel row on top,
the profile curve with the threshold (orange) and detected edges (red) below.

Not simulated: exposure time. On the real sensor you set the integration time, and it
has to adapt to the hall lighting so white doesn't saturate and black stays dark.

## Tune without Gazebo

`tools/fast_sim.py` renders the camera view straight from the track texture (same pinhole
model as the Gazebo camera), runs the same `lane_tracker.py` and drives a bicycle model.
A lap takes a few seconds, so it is the quick way to try gains and camera mounts:

```bash
cd src/NXP_Cup_Nordsee_Nitro/ros/nxp_cup_control
python3 tools/fast_sim.py --laps 2
python3 tools/fast_sim.py --v-max 2.5 --v-min 1.2 --video lap.mp4
python3 tools/fast_sim.py --y-off 0.12 --yaw-off -0.3      # crooked start
python3 tools/fast_sim.py --cam-h 0.20 --pitch 0.6        # try another camera mount
python3 tools/fast_sim.py --fps 15 --latency 0.15         # slow camera / delayed commands
python3 tools/fast_sim.py --camera linescan               # TSL1401
python3 tools/fast_sim.py --camera linescan --vignette 0.5  # with a cheap lens
python3 -m pytest test                                     # unit tests
```

Its drive model copies how Gazebo's Ackermann plugin turns `/cmd_vel` into a steering
angle. It fails a run the way the rulebook does: more than half the wheels off the track, or
leaving the crossing on the wrong branch. It has no tyre slip, so its top speed is
optimistic; confirm in Gazebo, then on the real car.

## Other track layouts

```bash
cd ros/nxp_cup_gazebo
python3 scripts/generate_track.py --list
python3 scripts/generate_track.py --layout oval
```

Layouts are segment lists at the top of `generate_track.py` (straights, arcs, chicanes).
The script refuses layouts that don't close. It rewrites the track model and
`config/track_info.yaml` (spawn pose, stop-cube pose); rebuild afterwards. The rules say the
race layout is only revealed on the day, so test on more than one.

## Match the real car

The car dimensions in `nxp_cup_car.urdf.xacro` are placeholders near the 2025 3D-printed kit.
Measure the real car and update:

- wheel base, wheel separation, wheel radius, steering limit (xacro properties)
- camera height, pitch and field of view: the `camera:` section of
  `config/line_follower.yaml` (or `line_follower_linescan.yaml`). The launch file mounts the
  simulated camera from there, and the tracker predicts lane width from it.

## How the line follower works

1. Average 3 image rows at a near and a far scan line into a brightness profile.
2. Find the inner edges of both black lines, searching only near last frame's positions.
   Pairs with the wrong spacing (start-line dashes, glare) are thrown away.
3. One line missing: infer it from the lane width. Both missing (crossing): keep the last
   command for 0.6 s, then stop.
4. Steering = PD on the near-row offset + feed-forward from the far-row offset.
   Speed drops as the far row sees a curve coming.

Steps 1-3 need no floating point per pixel, so they port to C on the MCU almost line for line.

## Not done yet

- Stopping within 10 cm of the "end of race" cube (worth +150 points)
- Lap timer at the start/finish line
- More layouts: 45° crossing, rulebook example layout
