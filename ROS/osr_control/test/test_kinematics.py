"""Unit tests for shared OSR rocker-bogie kinematics."""

import math

import pytest

from osr_control.kinematics import RoverDimensions, RoverKinematics


DEFAULT = RoverDimensions(
    d1=0.177,
    d2=0.310,
    d3=0.274,
    d4=0.253,
    wheel_radius=0.075,
)


@pytest.fixture
def kin():
    return RoverKinematics(DEFAULT, drive_no_load_rpm=223.0)


def test_straight_drive_equal_speed_opposite_sides(kin):
    cmd = kin.calculate_drive_velocities(0.5, kin.max_radius)
    omega = 0.5 / DEFAULT.wheel_radius
    assert cmd.left_front_vel == pytest.approx(omega)
    assert cmd.left_middle_vel == pytest.approx(omega)
    assert cmd.left_back_vel == pytest.approx(omega)
    assert cmd.right_front_vel == pytest.approx(-omega)
    assert cmd.right_middle_vel == pytest.approx(-omega)
    assert cmd.right_back_vel == pytest.approx(-omega)


def test_zero_speed_returns_zeros(kin):
    cmd = kin.calculate_drive_velocities(0.0, 1.0)
    assert cmd == type(cmd)()


def test_corner_positions_use_closest_and_farthest(kin):
    radius = 1.0
    corners = kin.calculate_corner_positions(radius)
    closest = math.atan2(DEFAULT.d3, abs(radius) - DEFAULT.d1)
    farthest = math.atan2(DEFAULT.d3, abs(radius) + DEFAULT.d1)
    assert corners.left_front_pos == pytest.approx(-closest)
    assert corners.left_back_pos == pytest.approx(closest)
    assert corners.right_front_pos == pytest.approx(-farthest)
    assert corners.right_back_pos == pytest.approx(farthest)
    assert abs(closest) != pytest.approx(abs(farthest))


def test_corner_positions_straight(kin):
    from osr_control.kinematics import CornerCommand
    assert kin.calculate_corner_positions(kin.max_radius) == CornerCommand()
    assert kin.calculate_corner_positions(-kin.max_radius) == CornerCommand()
    assert kin.calculate_corner_positions(-(kin.max_radius + 0.1)) == CornerCommand()


def test_turning_left_inner_wheels_slower(kin):
    cmd = kin.calculate_drive_velocities(0.4, 1.0)
    assert abs(cmd.left_middle_vel) < abs(cmd.right_middle_vel)


def test_rotate_in_place_corner_symmetry(kin):
    corners, drive = kin.calculate_rotate_in_place_cmd(0.2)
    assert corners.left_front_pos == pytest.approx(math.atan(DEFAULT.d3 / DEFAULT.d1))
    assert corners.left_back_pos == pytest.approx(-corners.left_front_pos)
    assert corners.right_back_pos == pytest.approx(math.atan(DEFAULT.d2 / DEFAULT.d1))
    assert corners.right_front_pos == pytest.approx(-corners.right_back_pos)
    assert drive.left_front_vel == pytest.approx(drive.right_front_vel)
    assert drive.left_middle_vel == pytest.approx(drive.right_middle_vel)


def test_twist_to_radius_infinite_when_no_yaw(kin):
    assert math.isinf(kin.twist_to_turning_radius(0.5, 0.0))


def test_twist_to_radius_clips_to_min(kin):
    radius = kin.twist_to_turning_radius(0.1, 10.0)
    assert radius == pytest.approx(kin.min_radius)


def test_body_speed_for_radius_clamps_reverse(kin):
    commanded = -10.0
    limited = kin.body_speed_for_radius(commanded, kin.min_radius)
    forward_cap = kin.body_speed_for_radius(10.0, kin.min_radius)
    expected_cap = (
        abs(kin.min_radius) / (abs(kin.min_radius) + kin.d1) * kin.max_vel)
    assert limited < 0
    assert limited == pytest.approx(-expected_cap)
    assert abs(limited) == pytest.approx(forward_cap)
    assert abs(limited) < abs(commanded)


def test_intuitive_zero_linear_radius_is_clipped(kin):
    # Large yaw while stopped would yield |r| < min_radius without clipping
    radius = kin.twist_to_turning_radius(
        0.0, 100.0, intuitive_mode=True)
    assert abs(radius) == pytest.approx(kin.min_radius)

    # Tiny yaw while stopped would yield |r| > max_radius without clipping
    radius = kin.twist_to_turning_radius(
        0.0, 1e-6, intuitive_mode=True)
    assert abs(radius) == pytest.approx(kin.max_radius)


def test_non_finite_twist_is_rejected(kin):
    for bad in (float("nan"), float("inf"), float("-inf")):
        with pytest.raises(ValueError):
            kin.twist_to_turning_radius(bad, 1.0)
        with pytest.raises(ValueError):
            kin.twist_to_turning_radius(1.0, bad)
        with pytest.raises(ValueError):
            kin.calculate_rotate_in_place_cmd(bad)
        with pytest.raises(ValueError):
            kin.calculate_drive_velocities(bad, kin.max_radius)


def test_nan_radius_is_rejected(kin):
    with pytest.raises(ValueError):
        kin.calculate_corner_positions(float("nan"))
    with pytest.raises(ValueError):
        kin.calculate_drive_velocities(0.5, float("nan"))


def test_infinite_radius_still_drives_straight(kin):
    radius = kin.twist_to_turning_radius(0.5, 0.0)
    assert math.isinf(radius)

    cmd = kin.calculate_drive_velocities(0.5, radius)
    omega = 0.5 / DEFAULT.wheel_radius
    assert cmd.left_front_vel == pytest.approx(omega)
    assert cmd.right_front_vel == pytest.approx(-omega)
