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

from arguments import Arguments

class TestArguments(unittest.TestCase):

    def setUp(self):
        self.arguments = Arguments()

    def test_bluetooth_argument_is_false_by_default(self):
        """ Is the bluetooth argument False by default? """
        self.assertFalse(self.arguments.bt_flag)

    def test_unix_argument_is_false_by_default(self):
        """ Is the unix argument False by default? """
        self.assertFalse(self.arguments.unix_flag)

    def test_xbox_argument_is_false_by_default(self):
        """ Is the xbox argument False by default? """
        self.assertFalse(self.arguments.xbox_flag)

if __name__ == '__main__':
    unittest.main()