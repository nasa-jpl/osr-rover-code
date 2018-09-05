#!/usr/bin/env python3

"""
File location is appended to path
to allow testing of scripts in directory above.
Due to scripts being stand alone and not modules.
"""
import sys
import os.path
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__),
    os.path.pardir)))

import unittest

from connections import Connections

class TestConnections(unittest.TestCase):
    """ 
    Note: Requires hardware to complete writing tests
    """

    def setUp(self):
        self.config = {}
        self.connections = Connections(self.config)

    def test_connection_config_is_initialized(self):
        """ Is the bluetooth argument False by default? """
        self.assertTrue(self.connections.config == self.config)

    def test_connection_joystick_is_initialized_as_none(self):
        """ Is the joystick initialized as None? """
        self.assertIsNone(self.connections.joy)

    def test_connection_bluetooth_socket_is_initialized_as_none(self):
        """ Is the bluetooth socket initialized as None? """
        self.assertIsNone(self.connections.bt_sock)

    def test_connection_led_is_initialized_as_zero(self):
        """ Is the bluetooth socket initialized as zero (0)? """
        self.assertTrue(self.connections.led == 0)

    def test_connection_unix_socket_is_initialized_as_none(self):
        """ Is the unix socket initialized as None? """
        self.assertIsNone(self.connections.unix_sock)


if __name__ == '__main__':
    unittest.main()
