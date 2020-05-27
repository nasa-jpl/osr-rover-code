#!/usr/bin/env python
import serial
import math
import rospy

from roboclaw import Roboclaw

from sensor_msgs.msg import JointState
from osr_msgs.msg import CommandDrive, CommandCorner, Status


class RoboclawWrapper(object):
    """Interface between the roboclaw motor drivers and the higher level rover code"""

    def __init__(self):
        rospy.loginfo( "Initializing motor controllers")

        # initialize attributes
        self.rc = None
        self.err = [None] * 5
        self.address = []
        self.current_enc_vals = None
        self.corner_cmd_buffer = None
        self.drive_cmd_buffer = None

        self.roboclaw_mapping = rospy.get_param('~roboclaw_mapping')
        self.encoder_limits = {}
        self.establish_roboclaw_connections()
        self.killMotors()  # don't move at start
        self.setup_encoders()

        # save settings to non-volatile (permanent) memory
        for address in self.address:
            self.rc.WriteNVM(address)

        for address in self.address:
            self.rc.ReadNVM(address)

        self.corner_max_vel = 1000
        # corner motor acceleration
        # Even though the actual method takes longs (2*32-1), roboclaw blog says 2**15 is 100%
        accel_max = 2**15-1
        accel_rate = rospy.get_param('/corner_acceleration_factor', 0.8)
        self.corner_accel = int(accel_max * accel_rate)
        self.roboclaw_overflow = 2**15-1
        # drive motor acceleration
        accel_max = 2**15-1
        accel_rate = rospy.get_param('/drive_acceleration_factor', 0.5)
        self.acceleration = int(accel_max * accel_rate)
        self.errorCheck()

        self.killMotors()

        # set up publishers and subscribers
        self.corner_cmd_sub = rospy.Subscriber("/cmd_corner", CommandCorner, self.corner_cmd_cb, queue_size=1)
        self.drive_cmd_sub = rospy.Subscriber("/cmd_drive", CommandDrive, self.drive_cmd_cb, queue_size=1)
        self.enc_pub = rospy.Publisher("/encoder", JointState, queue_size=1)
        self.status_pub = rospy.Publisher("/status", Status, queue_size=1)

    def run(self):
        """Blocking loop which runs after initialization has completed"""
        rate = rospy.Rate(8)
 

        status = Status()

        counter = 0
        while not rospy.is_shutdown():

            # Check to see if there are commands in the buffer to send to the motor controller
            if self.drive_cmd_buffer:
                drive_fcn = self.send_drive_buffer_velocity
                drive_fcn(self.drive_cmd_buffer)
                self.drive_cmd_buffer = None
                
            if self.corner_cmd_buffer:
                self.send_corner_buffer(self.corner_cmd_buffer)
                self.corner_cmd_buffer = None

            # read from roboclaws and publish
            try:
                self.read_encoder_values()
                self.enc_pub.publish(self.current_enc_vals)
            except AssertionError as read_exc:
                rospy.logwarn( "Failed to read encoder values")

            # Downsample the rate of less important data
            if (counter >= 5):
                status.battery = self.getBattery()
                status.temp = self.getTemp()
                status.current = self.getCurrents()
                status.error_status = self.getErrors()
                counter = 0

            self.status_pub.publish(status)
            counter += 1
            rate.sleep()

    def establish_roboclaw_connections(self):
        """
        Attempt connecting to the roboclaws

        :raises Exception: when connection to one or more of the roboclaws is unsuccessful
        """
        self.rc = Roboclaw(rospy.get_param('/motor_controller/device', "/dev/serial0"),
                           rospy.get_param('/motor_controller/baud_rate', 115200))
        self.rc.Open()

        address_raw = rospy.get_param('motor_controller/addresses')
        address_list = (address_raw.split(','))
        self.address = [None]*len(address_list)
        for i in range(len(address_list)):
            self.address[i] = int(address_list[i])

        # initialize connection status to successful
        all_connected = True
        for address in self.address:
            rospy.logdebug("Attempting to talk to motor controller ''".format(address))
            version_response = self.rc.ReadVersion(address)
            connected = bool(version_response[0])
            if not connected:
                rospy.logerr("Unable to connect to roboclaw at '{}'".format(address))
                all_connected = False
            else:
                rospy.logdebug("Roboclaw version for address '{}': '{}'".format(address, version_response[1]))
        if all_connected:
            rospy.loginfo("Sucessfully connected to RoboClaw motor controllers")
        else:
            raise Exception("Unable to establish connection to one or more of the Roboclaw motor controllers")

    def setup_encoders(self):
        """Set up the encoders"""
        for motor_name, properties in self.roboclaw_mapping.iteritems():
            if "corner" in motor_name:
                enc_min, enc_max = self.read_encoder_limits(properties["address"], properties["channel"])
                self.encoder_limits[motor_name] = (enc_min, enc_max)
            else:
                self.encoder_limits[motor_name] = (None, None)
                self.rc.ResetEncoders(properties["address"])

    def read_encoder_values(self):
        """Query roboclaws and update current motors status in encoder ticks"""
        enc_msg = JointState()
        enc_msg.header.stamp = rospy.Time.now()
        for motor_name, properties in self.roboclaw_mapping.iteritems():
            enc_msg.name.append(motor_name)
            position = self.read_encoder_position(properties["address"], properties["channel"])
            velocity = self.read_encoder_velocity(properties["address"], properties["channel"])
            current = self.read_encoder_current(properties["address"], properties["channel"])
            enc_msg.position.append(self.tick2position(position,
                                                       self.encoder_limits[motor_name][0],
                                                       self.encoder_limits[motor_name][1],
                                                       properties['ticks_per_rev'],
                                                       properties['gear_ratio']))
            enc_msg.velocity.append(self.qpps2velocity(velocity,
                                                       properties['ticks_per_rev'],
                                                       properties['gear_ratio']))
            enc_msg.effort.append(current)

        self.current_enc_vals = enc_msg
        
    def corner_cmd_cb(self, cmd):
        """
        Takes the corner command and stores it in the buffer to be sent
        on the next iteration of the run() loop.
        """
        
        rospy.logdebug("Corner command callback received: {}".format(cmd))
        self.corner_cmd_buffer = cmd

    def send_corner_buffer(self, cmd):
        """
        Sends the corner command to the motor controller.
        """


        # convert position to tick
        encmin, encmax = self.encoder_limits["corner_left_front"]
        left_front_tick = self.position2tick(cmd.left_front_pos, encmin, encmax,
                                             self.roboclaw_mapping["corner_left_front"]["ticks_per_rev"],
                                             self.roboclaw_mapping["corner_left_front"]["gear_ratio"])
        encmin, encmax = self.encoder_limits["corner_left_back"]
        left_back_tick = self.position2tick(cmd.left_back_pos, encmin, encmax,
                                            self.roboclaw_mapping["corner_left_back"]["ticks_per_rev"],
                                            self.roboclaw_mapping["corner_left_back"]["gear_ratio"])
        encmin, encmax = self.encoder_limits["corner_right_back"]
        right_back_tick = self.position2tick(cmd.right_back_pos, encmin, encmax,
                                             self.roboclaw_mapping["corner_right_back"]["ticks_per_rev"],
                                             self.roboclaw_mapping["corner_right_back"]["gear_ratio"])
        encmin, encmax = self.encoder_limits["corner_right_front"]
        right_front_tick = self.position2tick(cmd.right_front_pos, encmin, encmax,
                                              self.roboclaw_mapping["corner_right_front"]["ticks_per_rev"],
                                              self.roboclaw_mapping["corner_right_front"]["gear_ratio"])

        self.send_position_cmd(self.roboclaw_mapping["corner_left_front"]["address"],
                               self.roboclaw_mapping["corner_left_front"]["channel"],
                               left_front_tick)
        self.send_position_cmd(self.roboclaw_mapping["corner_left_back"]["address"],
                               self.roboclaw_mapping["corner_left_back"]["channel"],
                               left_back_tick)
        self.send_position_cmd(self.roboclaw_mapping["corner_right_back"]["address"],
                               self.roboclaw_mapping["corner_right_back"]["channel"],
                               right_back_tick)
        self.send_position_cmd(self.roboclaw_mapping["corner_right_front"]["address"],
                               self.roboclaw_mapping["corner_right_front"]["channel"],
                               right_front_tick)

    def drive_cmd_cb(self, cmd):
        """
        Takes the drive command and stores it in the buffer to be sent
        on the next iteration of the run() loop.
        """
        
        rospy.logdebug("Drive command callback received: {}".format(cmd))
        self.drive_cmd_buffer = cmd

    def send_drive_buffer_velocity(self, cmd):
        """
        Sends the drive command to the motor controller, closed loop velocity commands
        """
        props = self.roboclaw_mapping["drive_left_front"]
        vel_cmd = self.velocity2qpps(cmd.left_front_vel, props["ticks_per_rev"], props["gear_ratio"])
        self.send_velocity_cmd(props["address"], props["channel"], vel_cmd)

        props = self.roboclaw_mapping["drive_left_middle"]
        vel_cmd = self.velocity2qpps(cmd.left_middle_vel, props["ticks_per_rev"], props["gear_ratio"])
        self.send_velocity_cmd(props["address"], props["channel"], vel_cmd)

        props = self.roboclaw_mapping["drive_left_back"]
        vel_cmd = self.velocity2qpps(cmd.left_back_vel, props["ticks_per_rev"], props["gear_ratio"])
        self.send_velocity_cmd(props["address"], props["channel"], vel_cmd)

        props = self.roboclaw_mapping["drive_right_back"]
        vel_cmd = self.velocity2qpps(cmd.right_back_vel, props["ticks_per_rev"], props["gear_ratio"])
        self.send_velocity_cmd(props["address"], props["channel"], vel_cmd)

        props = self.roboclaw_mapping["drive_right_middle"]
        vel_cmd = self.velocity2qpps(cmd.right_middle_vel, props["ticks_per_rev"], props["gear_ratio"])
        self.send_velocity_cmd(props["address"], props["channel"], vel_cmd)

        props = self.roboclaw_mapping["drive_right_front"]
        vel_cmd = self.velocity2qpps(cmd.right_front_vel, props["ticks_per_rev"], props["gear_ratio"])
        self.send_velocity_cmd(props["address"], props["channel"], vel_cmd)

    def send_position_cmd(self, address, channel, target_tick):
        """
        Wrapper around one of the send position commands

        :param address:
        :param channel:
        :param target_tick: int
        """
        cmd_args = [self.corner_accel, self.corner_max_vel, self.corner_accel, target_tick, 1]
        if channel == "M1":
            return self.rc.SpeedAccelDeccelPositionM1(address, *cmd_args)
        elif channel == "M2":
            return self.rc.SpeedAccelDeccelPositionM2(address, *cmd_args)
        else:
            raise AttributeError("Received unknown channel '{}'. Expected M1 or M2".format(channel))

    def read_encoder_position(self, address, channel):
        """Wrapper around self.rc.ReadEncM1 and self.rcReadEncM2 to simplify code"""
        if channel == "M1":
            val = self.rc.ReadEncM1(address)
        elif channel == "M2":
            val = self.rc.ReadEncM2(address)
        else:
            raise AttributeError("Received unknown channel '{}'. Expected M1 or M2".format(channel))

        assert val[0] == 1
        return val[1]


    def read_encoder_limits(self, address, channel):
        """Wrapper around self.rc.ReadPositionPID and returns subset of the data

        :return: (enc_min, enc_max)
        """
        if channel == "M1":
            result = self.rc.ReadM1PositionPID(address)
        elif channel == "M2":
            result = self.rc.ReadM2PositionPID(address)
        else:
            raise AttributeError("Received unknown channel '{}'. Expected M1 or M2".format(channel))

        assert result[0] == 1
        return (result[-2], result[-1])

    def send_velocity_cmd(self, address, channel, target_qpps):
        """
        Wrapper around one of the send velocity commands

        :param address:
        :param channel:
        :param target_qpps: int
        """
        # clip values
        target_qpps = max(-self.roboclaw_overflow, min(self.roboclaw_overflow, target_qpps))
        if channel == "M1":
            return self.rc.SpeedAccelM1(address, self.acceleration, target_qpps)
        elif channel == "M2":
            return self.rc.SpeedAccelM2(address, self.acceleration, target_qpps)
        else:
            raise AttributeError("Received unknown channel '{}'. Expected M1 or M2".format(channel))

    def read_encoder_velocity(self, address, channel):
        """Wrapper around self.rc.ReadSpeedM1 and self.rcReadSpeedM2 to simplify code"""
        if channel == "M1":
            val = self.rc.ReadSpeedM1(address)
        elif channel == "M2":
            val = self.rc.ReadSpeedM2(address)
        else:
            raise AttributeError("Received unknown channel '{}'. Expected M1 or M2".format(channel))

        assert val[0] == 1
        return val[1]

    def read_encoder_current(self, address, channel):
        """Wrapper around self.rc.ReadCurrents to simplify code"""
        if channel == "M1":
            return self.rc.ReadCurrents(address)[0]
        elif channel == "M2":
            return self.rc.ReadCurrents(address)[1]
        else:
            raise AttributeError("Received unknown channel '{}'. Expected M1 or M2".format(channel))

    def tick2position(self, tick, enc_min, enc_max, ticks_per_rev, gear_ratio):
        """
        Convert the absolute position from ticks to radian relative to the middle position

        :param tick:
        :param enc_min:
        :param enc_max:
        :param ticks_per_rev:
        :return:
        """
        ticks_per_rad = ticks_per_rev / (2 * math.pi)
        if enc_min is None or enc_max is None:
            return tick / ticks_per_rad
        mid = enc_min + (enc_max - enc_min) / 2

        # positive values correspond to the wheel turning left (z-axis points up)
        return -(tick - mid) / ticks_per_rad * gear_ratio

    def position2tick(self, position, enc_min, enc_max, ticks_per_rev, gear_ratio):
        """
        Convert the absolute position from radian relative to the middle position to ticks

                Clip values that are outside the range [enc_min, enc_max]

        :param position:
        :param enc_min:
        :param enc_max:
        :param ticks_per_rev:
        :return:
        """
        # positive values correspond to the wheel turning left (z-axis points up)
        position *= -1
        ticks_per_rad = ticks_per_rev / (2 * math.pi)
        if enc_min is None or enc_max is None:
            return position * ticks_per_rad
        mid = enc_min + (enc_max - enc_min) / 2
        tick = int(mid + position * ticks_per_rad / gear_ratio)

        return max(enc_min, min(enc_max, tick))

    def qpps2velocity(self, qpps, ticks_per_rev, gear_ratio):
        """
        Convert the given quadrature pulses per second to radian/s

        :param qpps: int
        :param ticks_per_rev:
        :param gear_ratio:
        :return:
        """
        return qpps / (2 * math.pi * gear_ratio * ticks_per_rev)

    def velocity2qpps(self, velocity, ticks_per_rev, gear_ratio):
        """
        Convert the given velocity to quadrature pulses per second

        :param velocity: rad/s
        :param ticks_per_rev:
        :param gear_ratio:
        :return: int
        """
        return int(velocity * 2 * math.pi * gear_ratio * ticks_per_rev)

    def getBattery(self):
        return self.rc.ReadMainBatteryVoltage(self.address[0])[1]

    def getTemp(self):
        temp = [None] * 5
        for i in range(5):
            temp[i] = self.rc.ReadTemp(self.address[i])[1]
        return temp

    def getCurrents(self):
        currents = [None] * 10
        for i in range(5):
            currs = self.rc.ReadCurrents(self.address[i])
            currents[2*i] = currs[1]
            currents[(2*i) + 1] = currs[2]
        return currents

    def getErrors(self):
        return self.err

    def killMotors(self):
        """Stops all motors on Rover"""
        for i in range(5):
            self.rc.ForwardM1(self.address[i], 0)
            self.rc.ForwardM2(self.address[i], 0)

    def errorCheck(self):
        """Checks error status of each motor controller, returns 0 if any errors occur"""
        for i in range(len(self.address)):
            self.err[i] = self.rc.ReadError(self.address[i])[1]
        for error in self.err:
            if error:
                self.killMotors()
                rospy.logerr("Motor controller Error: \n'{}'".format(error))


if __name__ == "__main__":
    rospy.init_node("roboclaw wrapper", log_level=rospy.INFO)
    rospy.loginfo("Starting the roboclaw wrapper node")

    wrapper = RoboclawWrapper()
    rospy.on_shutdown(wrapper.killMotors)
    wrapper.run()
