"""Unit tests for setting the drive velocity PID gains at run time."""

import pytest
from rclpy.parameter import Parameter

from osr_control.roboclaw import Roboclaw
from osr_control.roboclaw_wrapper import RoboclawWrapper


class FakeRoboclaw:
    """Records velocity PID writes"""

    def __init__(self):
        self.calls = []

    def SetM1VelocityPID(self, address, p, i, d, qpps):
        self.calls.append(("M1", address, p, i, d, qpps))
        return True

    def SetM2VelocityPID(self, address, p, i, d, qpps):
        self.calls.append(("M2", address, p, i, d, qpps))
        return True


class FakeLogger:
    def info(self, msg):
        pass

    def warn(self, msg):
        pass


@pytest.fixture
def wrapper():
    # skip Node.__init__, which would open the serial port
    wrapper = RoboclawWrapper.__new__(RoboclawWrapper)
    wrapper.rc = FakeRoboclaw()
    wrapper.log = FakeLogger()
    wrapper.drive_velocity_pid = None
    wrapper.roboclaw_mapping = {
        "drive_left_front": {"address": 128, "channel": "M1"},
        "drive_left_middle": {"address": 128, "channel": "M2"},
    }
    return wrapper


def test_parameter_sets_pid_on_every_drive_motor(wrapper):
    param = Parameter("drive_velocity_pid", Parameter.Type.DOUBLE_ARRAY, [1.5, 0.25, 0.0, 3000.0])
    result = wrapper.parameters_callback([param])
    assert result.successful
    assert wrapper.drive_velocity_pid == [1.5, 0.25, 0.0, 3000.0]
    assert wrapper.rc.calls == [
        ("M1", 128, 1.5, 0.25, 0.0, 3000),
        ("M2", 128, 1.5, 0.25, 0.0, 3000),
    ]


def test_parameter_before_connection_is_stored_only(wrapper):
    wrapper.rc = None
    param = Parameter("drive_velocity_pid", Parameter.Type.DOUBLE_ARRAY, [1.0, 0.5, 0.0, 2000.0])
    assert wrapper.parameters_callback([param]).successful
    assert wrapper.drive_velocity_pid == [1.0, 0.5, 0.0, 2000.0]


def test_parameter_rejects_wrong_length(wrapper):
    param = Parameter("drive_velocity_pid", Parameter.Type.DOUBLE_ARRAY, [1.0, 0.5, 0.0])
    result = wrapper.parameters_callback([param])
    assert not result.successful
    assert wrapper.rc.calls == []


class FakePort:
    def __init__(self):
        self.written = b""

    def write(self, data):
        self.written += data

    def read(self, n):
        return b"\xff"  # ack

    def flushInput(self):
        pass


def test_set_velocity_pid_accepts_float_gains():
    rc = Roboclaw("/dev/null", 115200)
    rc._port = FakePort()
    assert rc.SetM1VelocityPID(128, 1.5, 0.25, 0.0, 3000)
    # address, command 28, then D, P, I as 16.16 fixed point and QPPS
    assert rc._port.written[:18] == bytes([128, 28]) + (0).to_bytes(4, "big") \
        + (98304).to_bytes(4, "big") + (16384).to_bytes(4, "big") + (3000).to_bytes(4, "big")
