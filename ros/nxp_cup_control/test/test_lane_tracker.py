"""Unit tests for the ROS-free line follower core (run with pytest)."""
import numpy as np

from nxp_cup_control.lane_tracker import CameraGeometry, LineFollower

W, H = 320, 240


def synthetic(center_shift_px=0, lane=None):
    """Straight lane: white road, black edge lines, grey floor outside."""
    img = np.full((H, W), 95, np.uint8)
    geo = CameraGeometry()
    for r in range(H):
        try:
            half = geo.lane_px(r / (H - 1), W, H) / 2 if lane is None else lane / 2
        except ValueError:
            continue
        c = W / 2 + center_shift_px
        lo, hi = int(c - half), int(c + half)
        img[r, max(0, lo - 6):max(0, lo)] = 20
        img[r, max(0, lo):max(0, min(W, hi))] = 245
        img[r, max(0, hi):min(W, hi + 6)] = 20
    return img


def test_centered_goes_straight():
    f = LineFollower()
    for _ in range(30):     # speed ramps up at accel_limit
        v, steer = f.step(synthetic(), 1 / 30)
    assert v > 0.9 * f.cp.v_max
    assert abs(steer) < 0.02


def test_lane_to_the_right_steers_right():
    f = LineFollower()
    for _ in range(5):
        _, steer = f.step(synthetic(+40), 1 / 30)
    assert steer < -0.05      # negative = right turn (ROS convention)


def test_all_white_holds_then_stops():
    f = LineFollower()
    for _ in range(5):
        f.step(synthetic(), 1 / 30)
    white = np.full((H, W), 245, np.uint8)
    v, _ = f.step(white, 1 / 30)
    assert v > 0               # crossing: keep going on the last command
    for _ in range(60):
        v, _ = f.step(white, 1 / 30)
    assert v == 0.0            # lost for longer than hold_time: stop


def test_one_line_out_of_view_still_initialises():
    f = LineFollower()
    v, _ = f.step(synthetic(+120), 1 / 30)   # right line is off the image
    assert f.near.last.measured >= 1
    assert v > 0
