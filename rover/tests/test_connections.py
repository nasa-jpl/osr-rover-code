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

from unittest.mock import patch
from unittest import TestCase

from mocks import MockBlueTooth


class BluetoothConnectionTest(TestCase):

    def setUp(self):
        # mocked bluetooth module to avoid
        # install issues between development environments
        # on OSX installing bluetooth is a chore.
        # mocking provides same outcome and avoids pain :)
        modules = {'bluetooth': MockBlueTooth}
        self.module_patcher = patch.dict("sys.modules", modules)
        self.module_patcher.start()
        from connections import Connections
        
        self.config = {}
        self.config['BLUETOOTH_SOCKET_CONFIG'] = {}
        self.config['BLUETOOTH_SOCKET_CONFIG']['UUID'] = '123'
        self.config['BLUETOOTH_SOCKET_CONFIG']['name'] = 'test'
        self.conn = Connections(self.config)
    
    @patch('bluetooth.BluetoothSocket')
    def test_btConnect(self, mock_bt):
        self.conn._btConnect()
        self.assertTrue(type(self.conn.bt_sock))

    def tearDown(self):
        self.module_patcher.stop()


if __name__ == '__main__':
    unittest.main()
