"""Unit tests for RoboclawWrapper helpers that don't need a serial port."""

import pytest

from osr_control.roboclaw_wrapper import RoboclawWrapper


class FakeRoboclaw:
    """Returns canned replies in the same format as osr_control.roboclaw.Roboclaw"""

    def ReadCurrents(self, address):
        # (success, M1 current, M2 current) in units of 10 mA
        return (1, 152, 37)


@pytest.fixture
def wrapper():
    # skip Node.__init__, which would open the serial port
    wrapper = RoboclawWrapper.__new__(RoboclawWrapper)
    wrapper.rc = FakeRoboclaw()
    return wrapper


def test_read_encoder_current_m1(wrapper):
    assert wrapper.read_encoder_current(128, "M1") == pytest.approx(1.52)


def test_read_encoder_current_m2(wrapper):
    assert wrapper.read_encoder_current(128, "M2") == pytest.approx(0.37)


def test_read_encoder_current_unknown_channel(wrapper):
    with pytest.raises(AttributeError):
        wrapper.read_encoder_current(128, "M3")
