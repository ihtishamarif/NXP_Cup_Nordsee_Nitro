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
```

The lap monitor runs by default. It prints `LAP n: xx.xx s` in the launch terminal, and
when the car goes off the track (rulebook: more than half the wheels off, or wrong exit at
the crossing) it saves a report to `~/nxp_cup_runs/<date-time>/`:

- `failure.mp4`: the last 4 s of the follower's debug view, with commanded vs actual
  steering angle on every frame
- `log.csv`: the last 10 s of true pose, distance from the centre line, speed and steering
- `summary.txt`: where and why

Reset the world in Gazebo to start a new attempt; the monitor re-arms itself.

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
- camera height, pitch and field of view: xacro args **and** `camera:` in
  `config/line_follower.yaml` (the tracker predicts lane width from them)

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
