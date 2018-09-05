#!/usr/bin/env python3
import unittest
import json
import os.path


class TestConfig(unittest.TestCase):

    def setUp(self):
        filename = 'config.json'
        dir_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__),
            os.path.pardir)
        )
        config_path = f'{dir_path}{os.path.sep}{filename}'
        with open(config_path, 'r') as f:
           self.config = json.loads(f.read())

    def test_config_is_loaded(self):
       """ Is the config file loaded? """
       self.assertTrue(self.config)

    def test_config_has_battery_config(self):
       """ Does the config file have the battery configuration data? """
       self.assertTrue(self.config.get('BATTERY_CONFIG'))

    def test_config_battery_config_has_low_voltage_setting(self):
       """
       Does the config file have
       the battery configuration low voltage setting?
       """
       self.assertTrue(self.config.get('BATTERY_CONFIG').get('low_voltage'))

    def test_config_battery_config_has_high_voltage_setting(self):
       """ 
       Does the config file have
       the battery configuration high voltage setting?
       """
       self.assertTrue(self.config.get('BATTERY_CONFIG').get('high_voltage'))

    def test_config_has_motor_config(self):
       """ Does the config file have the motor configuration data? """
       self.assertTrue(self.config.get('MOTOR_CONFIG'))

    def test_config_motor_config_has_max_drive_current_setting(self):
       """ 
       Does the config file have
       the motor configuration max drive current setting?
       """
       self.assertTrue(self.config.get(
           'MOTOR_CONFIG').get('max_drive_current'))

    def test_config_motor_config_has_max_corner_current_setting(self):
       """ 
       Does the config file have
       the motor configuration max corner current setting?
       """
       self.assertTrue(self.config.get(
           'MOTOR_CONFIG').get('max_corner_current'))

    def test_config_motor_config_has_controller_address_setting(self):
       """ 
       Does the config file have
       the controller address setting?
       """
       self.assertTrue(self.config.get(
           'MOTOR_CONFIG').get('controller_address'))

    def test_config_has_controller_config(self):
       """ Does the config file have the controller configuration data? """
       self.assertTrue(self.config.get('CONTROLLER_CONFIG'))

    def test_config_controller_config_has_device_setting(self):
       """ 
       Does the config file have the device setting?
       """
       self.assertTrue(self.config.get('CONTROLLER_CONFIG').get('device'))

    def test_config_controller_config_has_baud_rate_setting(self):
       """ 
       Does the config file have the baud rate setting?
       """
       self.assertTrue(self.config.get('CONTROLLER_CONFIG').get('baud_rate'))

    def test_config_has_mechanical_config(self):
       """ Does the config file have the mechanical configuration data? """
       self.assertTrue(self.config.get('MECHANICAL_CONFIG'))

    """
    Note:
        d1, d2, d3, d4 all refer to the four corners of
        the robot that have the ability to pivot in order
        to turn the robot.

    """

    def test_config_mechanical_config_has_d1_setting(self):
       """ 
       Does the config file have the d1 setting?
       Note: d1 has to do with the turning ratio of the robot.
       """
       self.assertTrue(self.config.get('MECHANICAL_CONFIG').get('d1'))

    def test_config_mechanical_config_has_d2_setting(self):
       """ 
       Does the config file have the d1 setting?
       Note: d2 has to do with the turning ratio of the robot.
       """
       self.assertTrue(self.config.get('MECHANICAL_CONFIG').get('d2'))

    def test_config_mechanical_config_has_d3_setting(self):
       """ 
       Does the config file have the d1 setting?
       Note: d3 has to do with the turning ratio of the robot.
       """
       self.assertTrue(self.config.get('MECHANICAL_CONFIG').get('d3'))

    def test_config_mechanical_config_has_d4_setting(self):
       """ 
       Does the config file have the d1 setting?
       Note: d4 has to do with the turning ratio of the robot.
       """
       self.assertTrue(self.config.get('MECHANICAL_CONFIG').get('d4'))

    def test_config_mechanical_config_has_wheel_radius_setting(self):
       """ 
       Does the config file have the wheel radius setting?
       """
       self.assertTrue(self.config.get(
           'MECHANICAL_CONFIG').get('wheel_radius'))

    def test_config_has_bluetooth_socket_config(self):
       """ 
       Does the config file have
       the bluetooth socket configuration data?
       """
       self.assertTrue(self.config.get('BLUETOOTH_SOCKET_CONFIG'))

    def test_config_bluetooth_socket_config_has_uuid_setting(self):
       """ 
       Does the config file have the bluetooth socket uuid setting?
       """
       self.assertTrue(self.config.get('BLUETOOTH_SOCKET_CONFIG').get('UUID'))

    def test_config_bluetooth_socket_config_has_name_setting(self):
       """ 
       Does the config file have the bluetooth socket name setting?
       """
       self.assertTrue(self.config.get('BLUETOOTH_SOCKET_CONFIG').get('name'))

    def test_config_bluetooth_socket_config_has_timeout_setting(self):
       """ 
       Does the config file have the bluetooth socket timeout setting?
       """
       self.assertTrue(self.config.get(
           'BLUETOOTH_SOCKET_CONFIG').get('timeout'))

    def test_config_has_unix_socket_config(self):
       """ 
       Does the config file have
       the unix socket configuration data?
       """
       self.assertTrue(self.config.get('UNIX_SOCKET_CONFIG'))

    def test_config_unix_socket_config_has_path_setting(self):
       """ 
       Does the config file have the unix socket path setting?
       """
       self.assertTrue(self.config.get('UNIX_SOCKET_CONFIG').get('path'))

if __name__ == '__main__':
    unittest.main()
