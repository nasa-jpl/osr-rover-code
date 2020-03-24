#!/usr/bin/env python

import rospy
import math

from sensor_msgs.msg import JointState
from osr_msgs.msg import Joystick, CommandDrive, CommandCorner


class Rover(object):
    """Math and motor control algorithms to move the rover"""

    def __init__(self):
        distances = rospy.get_param('mech_dist','7.254,10.5,10.5,10.073').split(",")
        self.d1 = float(distances[0]) * 0.0254
        self.d2 = float(distances[1]) * 0.0254
        self.d3 = float(distances[2]) * 0.0254
        self.d4 = float(distances[3]) * 0.0254

        self.min_radius = 0.45
        self.max_radius = 6.4

        self.no_cmd_thresh = 0.05  # [rad]
        self.wheel_radius = rospy.get_param("/wheel_radius", 0.075)  # [m]
        drive_no_load_rpm = rospy.get_param("/drive_no_load_rpm", 130)
        self.max_vel = self.wheel_radius * drive_no_load_rpm / 60 * 2 * math.pi  # wheel radius * omega_no_load [m/s]

        rospy.Subscriber("/joystick", Joystick, self.cmd_cb)
        rospy.Subscriber("/encoder", JointState, self.enc_cb)

        self.corner_cmd_pub = rospy.Publisher("/cmd_corner", CommandCorner, queue_size=1)
        self.drive_cmd_pub = rospy.Publisher("/cmd_drive", CommandDrive, queue_size=1)

    def cmd_cb(self, msg):
        desired_turning_radius = self.calculate_turning_radius(msg.steering)
        rospy.logdebug("desired turning radius: {}".format(desired_turning_radius))
        corner_cmd_msg = self.calculate_corner_positions(desired_turning_radius)
        # temporarily convert (-50, 50) velocity range to actual velocity in m/s
        velocity = msg.vel * self.max_vel / 50
        rospy.logdebug("velocity drive cmd: {} m/s".format(velocity))
        # TODO shouldn't supply commanded steering, should supply current steering.
        drive_cmd_msg = self.calculate_drive_velocities(velocity, desired_turning_radius)
        rospy.logdebug("drive cmd: {}".format(drive_cmd_msg))
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

        :param speed: Drive speed command range from -max_vel to max_vel
        :param radius: Current turning radius in m
        """
        # clip the value to the maximum allowed velocity
        speed = max(-self.max_vel, min(self.max_vel, speed))
        cmd_msg = CommandDrive()
        if speed == 0:
            return cmd_msg

        elif abs(current_radius) >= self.max_radius:  # Very large turning radius, all wheels same speed
            cmd_msg.left_front_vel = speed
            cmd_msg.left_middle_vel = speed
            cmd_msg.left_back_vel = speed
            cmd_msg.right_back_vel = speed
            cmd_msg.right_middle_vel = speed
            cmd_msg.right_front_vel = speed

            return cmd_msg

        else:
            # the entire vehicle move with the same angular velocity dictated by the desired speed,
            # around the radius of the turn. v = r * omega
            angular_velocity_center = float(speed) / current_radius
            # calculate desired velocities of all centers of wheels. Corner wheels on the same side
            # move with the same velocity. v = r * omega again
            vel_middle_closest = (current_radius - self.d4) * angular_velocity_center
            vel_corner_closest = (current_radius - self.d1) * angular_velocity_center
            vel_corner_farthest = (current_radius + self.d1) * angular_velocity_center
            vel_middle_farthest = (current_radius + self.d4) * angular_velocity_center

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

        theta_front_closest = math.atan2(self.d3, radius - self.d1)
        theta_front_farthest = math.atan2(self.d3, radius + self.d1)

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

    def calculate_turning_radius(self, direction):
        """
        Convert a range (-100, 100) to an actual turning radius

        :param direction: -100 is left, 100 is right
        :return: physical turning radius in meter, clipped to the rover's limits
        """
        max_theta_cl = math.pi/4
        # angle around z axis pointing up of wheel closest to center of circle
        theta_cl = -float(direction) / 100.0 * max_theta_cl

        try:
            radius = self.d1 + self.d3 / math.tan(theta_cl)
        except ZeroDivisionError:
            return float("Inf")
        
        # clip values so they lie in (-max_radius, -min_radius) or (min_radius, max_radius)
        if radius > 0:
            radius = max(self.min_radius, min(self.max_radius, radius))
        else:
            radius = max(-self.max_radius, min(-self.min_radius, radius))

        return radius


if __name__ == '__main__':
    rospy.init_node('rover', log_level=rospy.INFO)
    rospy.loginfo("Starting the rover node")
    rover = Rover()
    rospy.spin()
