"""Pure rocker-bogie kinematics for the OSR (no ROS dependencies).

Geometry matches the dimensions illustrated in dimensions_wheels_illustration.png:
  d1: half-track (y) from centerline to corner wheel centers
  d2: longitudinal distance from center to rear corner wheels
  d3: longitudinal distance from center to front corner wheels
  d4: half-track (y) from centerline to middle wheel centers
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class RoverDimensions:
    d1: float
    d2: float
    d3: float
    d4: float
    wheel_radius: float


@dataclass(frozen=True)
class DriveCommand:
    left_front_vel: float = 0.0
    left_middle_vel: float = 0.0
    left_back_vel: float = 0.0
    right_front_vel: float = 0.0
    right_middle_vel: float = 0.0
    right_back_vel: float = 0.0


@dataclass(frozen=True)
class CornerCommand:
    left_front_pos: float = 0.0
    left_back_pos: float = 0.0
    right_front_pos: float = 0.0
    right_back_pos: float = 0.0


class RoverKinematics:
    """Inverse kinematics: twist / turning radius → wheel speeds and corner angles."""

    def __init__(
        self,
        dimensions: RoverDimensions,
        min_radius: float = 0.45,
        max_radius: float = 6.4,
        max_vel: Optional[float] = None,
        drive_no_load_rpm: Optional[float] = None,
    ):
        self.dimensions = dimensions
        self.min_radius = min_radius
        self.max_radius = max_radius
        if max_vel is not None:
            self.max_vel = max_vel
        elif drive_no_load_rpm is not None:
            self.max_vel = (
                dimensions.wheel_radius * drive_no_load_rpm / 60.0 * 2.0 * math.pi
            )
        else:
            self.max_vel = float("inf")

    @property
    def d1(self) -> float:
        return self.dimensions.d1

    @property
    def d2(self) -> float:
        return self.dimensions.d2

    @property
    def d3(self) -> float:
        return self.dimensions.d3

    @property
    def d4(self) -> float:
        return self.dimensions.d4

    @property
    def wheel_radius(self) -> float:
        return self.dimensions.wheel_radius

    def twist_to_turning_radius(
        self,
        linear_x: float,
        angular_z: float,
        clip: bool = True,
        intuitive_mode: bool = False,
    ) -> float:
        """Convert commanded twist into a turning radius [m]."""
        try:
            if intuitive_mode and linear_x < 0:
                radius = linear_x / -angular_z
            else:
                radius = linear_x / angular_z
        except ZeroDivisionError:
            return float("inf")

        if not clip:
            return radius

        if radius == 0:
            if not intuitive_mode or angular_z == 0:
                return self.max_radius
            # Proxy radius while standing still; still clip to rover limits below.
            radius = self.min_radius * self.max_vel / angular_z

        if radius > 0:
            return max(self.min_radius, min(self.max_radius, radius))
        return max(-self.max_radius, min(-self.min_radius, radius))

    def calculate_corner_positions(self, radius: float) -> CornerCommand:
        """Corner angles [rad] for a turning radius. Positive radius = turn left."""
        if abs(radius) >= self.max_radius:
            return CornerCommand()

        theta_front_closest = math.atan2(self.d3, abs(radius) - self.d1)
        theta_front_farthest = math.atan2(self.d3, abs(radius) + self.d1)

        if radius > 0:
            return CornerCommand(
                left_front_pos=-theta_front_closest,
                left_back_pos=theta_front_closest,
                right_back_pos=theta_front_farthest,
                right_front_pos=-theta_front_farthest,
            )
        return CornerCommand(
            left_front_pos=theta_front_farthest,
            left_back_pos=-theta_front_farthest,
            right_back_pos=-theta_front_closest,
            right_front_pos=theta_front_closest,
        )

    def calculate_drive_velocities(self, speed: float, current_radius: float) -> DriveCommand:
        """Wheel angular velocities [rad/s] for body speed [m/s] and turning radius [m]."""
        speed = max(-self.max_vel, min(self.max_vel, speed))
        if speed == 0:
            return DriveCommand()

        if abs(current_radius) >= self.max_radius:
            angular_vel = speed / self.wheel_radius
            return DriveCommand(
                left_front_vel=angular_vel,
                left_middle_vel=angular_vel,
                left_back_vel=angular_vel,
                right_back_vel=-angular_vel,
                right_middle_vel=-angular_vel,
                right_front_vel=-angular_vel,
            )

        radius = abs(current_radius)
        angular_velocity_center = float(speed) / radius
        vel_middle_closest = (radius - self.d4) * angular_velocity_center
        vel_corner_closest = math.hypot(radius - self.d1, self.d3) * angular_velocity_center
        vel_corner_farthest = math.hypot(radius + self.d1, self.d3) * angular_velocity_center
        vel_middle_farthest = (radius + self.d4) * angular_velocity_center

        ang_vel_middle_closest = vel_middle_closest / self.wheel_radius
        ang_vel_corner_closest = vel_corner_closest / self.wheel_radius
        ang_vel_corner_farthest = vel_corner_farthest / self.wheel_radius
        ang_vel_middle_farthest = vel_middle_farthest / self.wheel_radius

        if current_radius > 0:
            return DriveCommand(
                left_front_vel=ang_vel_corner_closest,
                left_back_vel=ang_vel_corner_closest,
                left_middle_vel=ang_vel_middle_closest,
                right_back_vel=-ang_vel_corner_farthest,
                right_front_vel=-ang_vel_corner_farthest,
                right_middle_vel=-ang_vel_middle_farthest,
            )
        return DriveCommand(
            left_front_vel=ang_vel_corner_farthest,
            left_back_vel=ang_vel_corner_farthest,
            left_middle_vel=ang_vel_middle_farthest,
            right_back_vel=-ang_vel_corner_closest,
            right_front_vel=-ang_vel_corner_closest,
            right_middle_vel=-ang_vel_middle_closest,
        )

    def calculate_rotate_in_place_cmd(self, angular_y: float) -> Tuple[CornerCommand, DriveCommand]:
        """Corner angles and drive speeds for rotating in place (angular about body z via pitch twist)."""
        corner = CornerCommand(
            left_front_pos=math.atan(self.d3 / self.d1),
            left_back_pos=-math.atan(self.d2 / self.d1),
            right_back_pos=math.atan(self.d2 / self.d1),
            right_front_pos=-math.atan(self.d3 / self.d1),
        )
        front_wheel_vel = math.hypot(self.d1, self.d3) * angular_y / self.wheel_radius
        back_wheel_vel = math.hypot(self.d1, self.d2) * angular_y / self.wheel_radius
        middle_wheel_vel = self.d4 * angular_y / self.wheel_radius
        drive = DriveCommand(
            left_front_vel=front_wheel_vel,
            right_front_vel=front_wheel_vel,
            left_back_vel=back_wheel_vel,
            right_back_vel=back_wheel_vel,
            left_middle_vel=middle_wheel_vel,
            right_middle_vel=middle_wheel_vel,
        )
        return corner, drive

    def body_speed_for_radius(self, commanded_linear_x: float, turning_radius: float) -> float:
        """Clamp commanded body speed so every drive wheel stays within max_vel."""
        radius = abs(turning_radius)
        # Outermost wheel path, using the same distances as calculate_drive_velocities
        farthest = max(math.hypot(radius + self.d1, self.d3), radius + self.d4)
        max_vel = radius / farthest * self.max_vel
        if math.isnan(max_vel):
            max_vel = self.max_vel
        limited = min(max_vel, abs(commanded_linear_x))
        if commanded_linear_x == 0.0:
            return 0.0
        return math.copysign(limited, commanded_linear_x)

    def angle_to_turning_radius(self, angle: float) -> float:
        """Virtual mid-front wheel angle → turning radius [m]."""
        try:
            return self.d3 / math.tan(angle)
        except ZeroDivisionError:
            return float("inf")
