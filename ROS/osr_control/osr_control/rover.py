import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from osr_interfaces.msg import CommandDrive, CommandCorner


class Rover(Node):
    """Math and motor control algorithms to move the rover"""

    def __init__(self):
        super().__init__("rover")
        self.get_logger().info("Initializing Rover")

        self.declare_parameter("/rover_dimensions")
        rover_dimensions = self.get_parameter('/rover_dimensions').get_parameter_value()
        self.d1 = rover_dimensions["d1"]
        self.d2 = rover_dimensions["d2"]
        self.d3 = rover_dimensions["d3"]
        self.d4 = rover_dimensions["d4"]

        self.min_radius = 0.45  # [m]
        self.max_radius = 6.4  # [m]

        self.no_cmd_thresh = 0.05  # [rad]
        self.declare_parameter("/rover_dimensions/wheel_radius")
        self.declare_parameter("/drive_no_load_rpm")
        self.wheel_radius = self.get_parameter(
            "/rover_dimensions/wheel_radius").get_parameter_value()  # [m]
        drive_no_load_rpm = self.get_parameter(
            "/drive_no_load_rpm").get_parameter_value()
        self.max_vel = self.wheel_radius * drive_no_load_rpm / 60 * 2 * math.pi  # [m/s]

        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", 
                                                    self.cmd_cb, queue_size=1)
        self.encoder_sub = self.create_subscription(JointState, "/encoder", 
                                                    self.enc_cb, queue_size=1)

        self.corner_cmd_pub = self.create_publisher(CommandCorner, 
                                                    "/cmd_corner", queue_size=1)
        self.drive_cmd_pub = self.create_publisher(CommandDrive, 
                                                   "/cmd_drive", queue_size=1)

    def cmd_cb(self, twist_msg):
        desired_turning_radius = self.calculate_turning_radius(twist_msg.angular.z)
        self.get_logger().debug("desired turning radius: " +
                                "{}".format(desired_turning_radius))
        corner_cmd_msg = self.calculate_corner_positions(desired_turning_radius)

        # if we're turning, calculate the max velocity the middle of the rover can go
        max_vel = abs(desired_turning_radius) / (abs(desired_turning_radius) + self.d1) * self.max_vel
        if math.isnan(max_vel):  # turning radius infinite, going straight
            max_vel = self.max_vel
        velocity = min(max_vel, twist_msg.linear.x)
        self.get_logger().debug("velocity drive cmd: {} m/s".format(velocity))
        # TODO shouldn't supply commanded steering, should supply current steering.
        drive_cmd_msg = self.calculate_drive_velocities(velocity, desired_turning_radius)
        self.get_logger().debug("drive cmd:\n{}".format(drive_cmd_msg))
        self.get_logger().debug("corner cmd:\n{}".format(corner_cmd_msg)) 
        if self.corner_cmd_threshold(corner_cmd_msg):
            self.corner_cmd_pub.publish(corner_cmd_msg)
        self.drive_cmd_pub.publish(drive_cmd_msg)

    def enc_cb(self, msg):
        self.curr_positions = dict(zip(msg.name, msg.position))

    def corner_cmd_threshold(self, corner_cmd):
        try:
            if abs(corner_cmd.left_front_pos - self.curr_positions["corner_left_front"]) > self.no_cmd_thresh:
                return True
            elif abs(corner_cmd.left_back_pos - self.curr_positions["corner_left_back"]) > self.no_cmd_thresh:
                return True
            elif abs(corner_cmd.right_back_pos - self.curr_positions["corner_right_back"]) > self.no_cmd_thresh:
                return True
            elif abs(corner_cmd.right_front_pos - self.curr_positions["corner_right_front"]) > self.no_cmd_thresh:
                return True
            else:
                return False
        except AttributeError:  # haven't received current encoder positions yet
            return True

    def calculate_drive_velocities(self, speed, current_radius):
        """
        Calculate target velocities for the drive motors based on desired speed and current turning radius

        :param speed: Drive speed command range from -max_vel to max_vel, with max vel depending on the turning radius
        :param radius: Current turning radius in m
        """
        # clip the value to the maximum allowed velocity
        speed = max(-self.max_vel, min(self.max_vel, speed))
        cmd_msg = CommandDrive()
        if speed == 0:
            return cmd_msg

        elif abs(current_radius) >= self.max_radius:  # Very large turning radius, all wheels same speed
            angular_vel = speed / self.wheel_radius
            cmd_msg.left_front_vel = angular_vel
            cmd_msg.left_middle_vel = angular_vel
            cmd_msg.left_back_vel = angular_vel
            cmd_msg.right_back_vel = angular_vel
            cmd_msg.right_middle_vel = angular_vel
            cmd_msg.right_front_vel = angular_vel

            return cmd_msg

        else:
            # for the calculations, we assume positive radius (turn left) and adjust later
            radius = abs(current_radius)
            # the entire vehicle moves with the same angular velocity dictated by the desired speed,
            # around the radius of the turn. v = r * omega
            angular_velocity_center = float(speed) / radius
            # calculate desired velocities of all centers of wheels. Corner wheels on the same side
            # move with the same velocity. v = r * omega again
            vel_middle_closest = (radius - self.d4) * angular_velocity_center
            vel_corner_closest = math.hypot(radius - self.d1, self.d3) * angular_velocity_center
            vel_corner_farthest = math.hypot(radius + self.d1, self.d3) * angular_velocity_center
            vel_middle_farthest = (radius + self.d4) * angular_velocity_center

            # now from these desired velocities, calculate the desired angular velocity of each wheel
            # v = r * omega again
            ang_vel_middle_closest = vel_middle_closest / self.wheel_radius
            ang_vel_corner_closest = vel_corner_closest / self.wheel_radius
            ang_vel_corner_farthest = vel_corner_farthest / self.wheel_radius
            ang_vel_middle_farthest = vel_middle_farthest / self.wheel_radius

            if current_radius > 0:  # turning left
                cmd_msg.left_front_vel = ang_vel_corner_closest
                cmd_msg.left_back_vel = ang_vel_corner_closest
                cmd_msg.left_middle_vel = ang_vel_middle_closest
                cmd_msg.right_back_vel = ang_vel_corner_farthest
                cmd_msg.right_front_vel = ang_vel_corner_farthest
                cmd_msg.right_middle_vel = ang_vel_middle_farthest
            else:  # turning right
                cmd_msg.left_front_vel = ang_vel_corner_farthest
                cmd_msg.left_back_vel = ang_vel_corner_farthest
                cmd_msg.left_middle_vel = ang_vel_middle_farthest
                cmd_msg.right_back_vel = ang_vel_corner_closest
                cmd_msg.right_front_vel = ang_vel_corner_closest
                cmd_msg.right_middle_vel = ang_vel_middle_closest

            return cmd_msg

    def calculate_corner_positions(self, radius):
        """
        Takes a turning radius and computes the required angle for each corner motor

        A small turning radius means a sharp turn
        A large turning radius means mostly straight. Any radius larger than max_radius is essentially straight
        because of the encoders' resolution

        :param radius: positive value means turn left. 0.45 < abs(turning_radius) < inf
        """
        cmd_msg = CommandCorner()

        if radius >= self.max_radius:
            return cmd_msg  # assume straight

        theta_front_closest = math.atan2(self.d3, abs(radius) - self.d1)
        theta_front_farthest = math.atan2(self.d3, abs(radius) + self.d1)

        if radius > 0:
            cmd_msg.left_front_pos = theta_front_closest
            cmd_msg.left_back_pos = -theta_front_closest
            cmd_msg.right_back_pos = -theta_front_farthest
            cmd_msg.right_front_pos = theta_front_farthest
        else:
            cmd_msg.left_front_pos = -theta_front_farthest
            cmd_msg.left_back_pos = theta_front_farthest
            cmd_msg.right_back_pos = theta_front_closest
            cmd_msg.right_front_pos = -theta_front_closest

        return cmd_msg

    def calculate_turning_radius(self, angle):
        """
        Convert a commanded angle into an actual turning radius

        :param angle: angle around the vertical which expresses how much the rover will rotate if it moves forward
        :return: physical turning radius in meter, clipped to the rover's limits
        """
        try:
            radius = self.d3 / math.tan(angle)
        except ZeroDivisionError:
            return float("Inf")
        
        # clip values so they lie in (-max_radius, -min_radius) or (min_radius, max_radius)
        if radius > 0:
            radius = max(self.min_radius, min(self.max_radius, radius))
        else:
            radius = max(-self.max_radius, min(-self.min_radius, radius))

        return radius


def main(args=None):
    rclpy.init(args=args)

    rover = Rover()

    rclpy.spin(rover)
    rover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
