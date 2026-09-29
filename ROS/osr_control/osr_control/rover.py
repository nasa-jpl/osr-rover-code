import math
from functools import partial

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf2_ros

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TwistWithCovariance, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from osr_interfaces.msg import CommandDrive, CommandCorner

from osr_control.kinematics import CornerCommand, DriveCommand, RoverDimensions, RoverKinematics


class Rover(Node):
    """Math and motor control algorithms to move the rover"""

    def __init__(self):
        super().__init__("rover")
        self.log = self.get_logger()
        self.log.info("Initializing Rover")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('rover_dimensions.d1', Parameter.Type.DOUBLE),
                ('rover_dimensions.d2', Parameter.Type.DOUBLE),
                ('rover_dimensions.d3', Parameter.Type.DOUBLE),
                ('rover_dimensions.d4', Parameter.Type.DOUBLE),
                ('rover_dimensions.wheel_radius', Parameter.Type.DOUBLE),
                ('drive_no_load_rpm', Parameter.Type.DOUBLE),
                ('enable_odometry', Parameter.Type.BOOL),
                ('publish_transform', Parameter.Type.BOOL)
            ]
        )
        dimensions = RoverDimensions(
            d1=self.get_parameter('rover_dimensions.d1').get_parameter_value().double_value,
            d2=self.get_parameter('rover_dimensions.d2').get_parameter_value().double_value,
            d3=self.get_parameter('rover_dimensions.d3').get_parameter_value().double_value,
            d4=self.get_parameter('rover_dimensions.d4').get_parameter_value().double_value,
            wheel_radius=self.get_parameter(
                'rover_dimensions.wheel_radius').get_parameter_value().double_value,
        )
        drive_no_load_rpm = self.get_parameter(
            "drive_no_load_rpm").get_parameter_value().double_value
        self.kin = RoverKinematics(dimensions, drive_no_load_rpm=drive_no_load_rpm)

        # Convenience aliases used by forward kinematics / threshold helpers
        self.d1 = self.kin.d1
        self.d2 = self.kin.d2
        self.d3 = self.kin.d3
        self.d4 = self.kin.d4
        self.wheel_radius = self.kin.wheel_radius
        self.min_radius = self.kin.min_radius
        self.max_radius = self.kin.max_radius
        self.max_vel = self.kin.max_vel
        self.no_cmd_thresh = 0.05  # [rad]

        self.should_calculate_odom = self.get_parameter("enable_odometry").get_parameter_value().bool_value
        self.should_publish_transform = self.get_parameter("publish_transform").get_parameter_value().bool_value
        if self.should_calculate_odom:
            self.get_logger().info("Calculting wheel odometry and publishing to /odom topic")
            self.odometry = Odometry()
            self.odometry.header.stamp = self.get_clock().now().to_msg()
            self.odometry.header.frame_id = "odom"
            self.odometry.child_frame_id = "base_link"
            self.odometry.pose.pose.orientation.w = 1.
        self.curr_positions = {}
        self.curr_velocities = {}
        self.curr_twist = TwistWithCovariance()
        self.curr_turning_radius = self.max_radius

        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel",
                                                    partial(self.cmd_cb, intuitive=False), 1)
        self.cmd_vel_int_sub = self.create_subscription(Twist, "/cmd_vel_intuitive",
                                                        partial(self.cmd_cb, intuitive=True), 1)
        self.drive_enc_sub = self.create_subscription(JointState, "/drive_state", self.enc_cb, 1)
        self.corner_enc_sub = self.create_subscription(JointState, "/corner_state", self.enc_cb, 1)

        self.turning_radius_pub = self.create_publisher(Float64, "/turning_radius", 1)
        if self.should_calculate_odom:
            self.odometry_pub = self.create_publisher(Odometry, "/odom", 2)
            self.tf_pub = tf2_ros.TransformBroadcaster(self)

        self.corner_cmd_pub = self.create_publisher(CommandCorner, "/cmd_corner", 1)
        self.drive_cmd_pub = self.create_publisher(CommandDrive, "/cmd_drive", 1)

    @staticmethod
    def _to_drive_msg(cmd: DriveCommand) -> CommandDrive:
        msg = CommandDrive()
        msg.left_front_vel = cmd.left_front_vel
        msg.left_middle_vel = cmd.left_middle_vel
        msg.left_back_vel = cmd.left_back_vel
        msg.right_front_vel = cmd.right_front_vel
        msg.right_middle_vel = cmd.right_middle_vel
        msg.right_back_vel = cmd.right_back_vel
        return msg

    @staticmethod
    def _to_corner_msg(cmd: CornerCommand) -> CommandCorner:
        msg = CommandCorner()
        msg.left_front_pos = cmd.left_front_pos
        msg.left_back_pos = cmd.left_back_pos
        msg.right_front_pos = cmd.right_front_pos
        msg.right_back_pos = cmd.right_back_pos
        return msg

    def cmd_cb(self, twist_msg, intuitive=False):
        """
        Respond to an incoming Twist command in one of two ways depending on the mode (intuitive)

        The Mathematically correct mode (intuitive=False) means that
         * when the linear velocity is zero, an angular velocity does not cause the corner motors to move
           (since simply steering the corners while standing still doesn't generate a twist)
         * when driving backwards, steering behaves opposite as what you intuitively might expect
           (this is to hold true to the commanded twist)
        Use this topic with a controller that generated velocities based on targets. When you're
        controlling the robot with a joystick or other manual input topic, consider using the
        /cmd_vel_intuitive topic instead.

        The Intuitive mode (intuitive=True) means that sending a positive angular velocity (moving joystick left)
        will always make the corner wheels turn 'left' regardless of the linear velocity.

        :param intuitive: determines the mode
        """
        try:
            if twist_msg.angular.y and not twist_msg.linear.x:
                corner_cmd, drive_cmd = self.kin.calculate_rotate_in_place_cmd(twist_msg.angular.y)
                corner_cmd_msg = self._to_corner_msg(corner_cmd)
                drive_cmd_msg = self._to_drive_msg(drive_cmd)
            else:
                desired_turning_radius = self.kin.twist_to_turning_radius(
                    twist_msg.linear.x, twist_msg.angular.z, intuitive_mode=intuitive)
                self.get_logger().debug(
                    "desired turning radius: {}".format(desired_turning_radius), throttle_duration_sec=1)
                corner_cmd_msg = self._to_corner_msg(
                    self.kin.calculate_corner_positions(desired_turning_radius))

                velocity = self.kin.body_speed_for_radius(twist_msg.linear.x, desired_turning_radius)
                self.get_logger().debug("velocity drive cmd: {} m/s".format(velocity), throttle_duration_sec=1)

                drive_cmd_msg = self._to_drive_msg(
                    self.kin.calculate_drive_velocities(velocity, desired_turning_radius))
        except ValueError as err:
            # Ignoring the message keeps the node up; without this the exception reaches rclpy
            # and the node exits, and respawn brings it back only to be killed by the next one.
            self.get_logger().warn(
                "ignoring twist command: {}".format(err), throttle_duration_sec=1)
            return

        self.get_logger().debug("drive cmd:\n{}".format(drive_cmd_msg), throttle_duration_sec=1)
        self.get_logger().debug("corner cmd:\n{}".format(corner_cmd_msg), throttle_duration_sec=1)

        self.corner_cmd_pub.publish(corner_cmd_msg)
        self.drive_cmd_pub.publish(drive_cmd_msg)

    def enc_cb(self, msg):
        """When we get a JointState message from the drive or corner motors"""
        self.curr_positions = {**self.curr_positions, **dict(zip(msg.name, msg.position))}
        self.curr_velocities = {**self.curr_velocities, **dict(zip(msg.name, msg.velocity))}
        if self.should_calculate_odom and len(self.curr_positions) == 10:
            now = self.get_clock().now()
            dt = float(now.nanoseconds - (self.odometry.header.stamp.sec*10**9 + self.odometry.header.stamp.nanosec)) / 10**9
            self.forward_kinematics()
            dx = self.curr_twist.twist.linear.x * dt
            dth = self.curr_twist.twist.angular.z * dt
            current_angle = 2 * math.atan2(self.odometry.pose.pose.orientation.z,
                                           self.odometry.pose.pose.orientation.w)
            new_angle = current_angle + dth
            self.odometry.pose.pose.orientation.z = math.sin(new_angle/2.)
            self.odometry.pose.pose.orientation.w = math.cos(new_angle/2.)
            self.odometry.pose.pose.position.x += math.cos(new_angle) * dx
            self.odometry.pose.pose.position.y += math.sin(new_angle) * dx
            self.odometry.pose.covariance = 36 * [0.0,]
            self.odometry.twist.covariance[0] = 0.0225
            self.odometry.twist.covariance[5] = 0.01
            self.odometry.twist.covariance[-5] = 0.0225
            self.odometry.twist.covariance[-1] = 0.04
            self.odometry.twist = self.curr_twist
            self.odometry.header.stamp = now.to_msg()
            self.odometry_pub.publish(self.odometry)
            if self.should_publish_transform:
                transform_msg = TransformStamped()
                transform_msg.header.frame_id = "odom"
                transform_msg.child_frame_id = "base_link"
                transform_msg.header.stamp = now.to_msg()
                transform_msg.transform.translation.x = self.odometry.pose.pose.position.x
                transform_msg.transform.translation.y = self.odometry.pose.pose.position.y
                transform_msg.transform.rotation = self.odometry.pose.pose.orientation
                self.tf_pub.sendTransform(transform_msg)

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
        except AttributeError:
            return True

    def forward_kinematics(self):
        """
        Calculate current twist of the rover given current drive and corner motor velocities
        Also approximate current turning radius.

        Note that forward kinematics means solving an overconstrained system since the corner
        motors may not be aligned perfectly and drive velocities might fight each other
        """
        theta_fl = -self.curr_positions['corner_left_front']
        theta_fr = -self.curr_positions['corner_right_front']
        theta_bl = -self.curr_positions['corner_left_back']
        theta_br = -self.curr_positions['corner_right_back']
        if theta_fl + theta_fr + theta_bl + theta_br > 0:
            r_front_closest = self.d1 + self.kin.angle_to_turning_radius(theta_fl)
            r_front_farthest = -self.d1 + self.kin.angle_to_turning_radius(theta_fr)
            r_back_closest = -self.d1 - self.kin.angle_to_turning_radius(theta_bl)
            r_back_farthest = self.d1 - self.kin.angle_to_turning_radius(theta_br)
        else:
            r_front_farthest = self.d1 + self.kin.angle_to_turning_radius(theta_fl)
            r_front_closest = -self.d1 + self.kin.angle_to_turning_radius(theta_fr)
            r_back_farthest = -self.d1 - self.kin.angle_to_turning_radius(theta_bl)
            r_back_closest = self.d1 - self.kin.angle_to_turning_radius(theta_br)
        approx_turning_radius = sum(sorted([r_front_farthest, r_front_closest, r_back_farthest, r_back_closest])[1:3])/2.0

        if math.isnan(approx_turning_radius):
            approx_turning_radius = self.max_radius
        self.get_logger().debug(
            "Current approximate turning radius: {}".format(round(approx_turning_radius, 2)),
            throttle_duration_sec=1)
        self.curr_turning_radius = approx_turning_radius

        drive_angular_velocity = (
            self.curr_velocities['drive_left_middle'] + self.curr_velocities['drive_right_middle']) / 2.
        self.curr_twist.twist.linear.x = drive_angular_velocity * self.wheel_radius
        try:
            self.curr_twist.twist.angular.z = self.curr_twist.twist.linear.x / self.curr_turning_radius
        except ZeroDivisionError:
            self.curr_twist.twist.linear.x = 0.0
            drive_angular_velocity = (
                self.curr_velocities['drive_left_middle'] - self.curr_velocities['drive_right_middle']) / 2.0
            self.curr_twist.twist.angular.z = drive_angular_velocity * self.wheel_radius / self.d4
            self.get_logger().debug(
                f"Turn-in-place detected. Angular velocity: {self.curr_twist.twist.angular.z}",
                throttle_duration_sec=1)


def main(args=None):
    rclpy.init(args=args)

    rover = Rover()

    rclpy.spin(rover)
    rover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
