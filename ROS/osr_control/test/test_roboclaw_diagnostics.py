"""Unit tests for the /diagnostics and /battery_state messages of RoboclawWrapper."""

import math

import pytest
from diagnostic_msgs.msg import DiagnosticStatus

from osr_control.roboclaw_wrapper import RoboclawWrapper
from osr_interfaces.msg import Status


@pytest.fixture
def wrapper():
    # skip Node.__init__, which would open the serial port
    wrapper = RoboclawWrapper.__new__(RoboclawWrapper)
    wrapper.address = [128, 129, 130]
    return wrapper


@pytest.fixture
def status():
    status = Status()
    status.battery = 12.3
    # no error, M1 over-current warning, temperature error
    status.error_status = ['0', '0x10000', '0x2']
    status.temp = [30.5, 41.0, 85.2]
    status.current = [1.5, 0.25, 2.0, 3.75, 0.0, 0.5]
    return status


def test_one_diagnostic_status_per_controller(wrapper, status):
    diagnostics = wrapper.build_diagnostics(status)
    assert [d.hardware_id for d in diagnostics.status] == ['128', '129', '130']
    assert diagnostics.status[0].name == 'roboclaw_wrapper: RoboClaw 128'


def test_diagnostic_levels_follow_error_bits(wrapper, status):
    diagnostics = wrapper.build_diagnostics(status)
    assert diagnostics.status[0].level == DiagnosticStatus.OK
    assert diagnostics.status[0].message == 'OK'
    assert diagnostics.status[1].level == DiagnosticStatus.WARN
    assert diagnostics.status[1].message == 'M1 Over-Current Warning'
    assert diagnostics.status[2].level == DiagnosticStatus.ERROR
    assert diagnostics.status[2].message == 'Temperature Error'


def test_diagnostic_values(wrapper, status):
    values = {kv.key: kv.value for kv in wrapper.build_diagnostics(status).status[1].values}
    assert values == {
        'Error code': '0x10000',
        'Temperature (C)': '41.0',
        'M1 current (A)': '2.00',
        'M2 current (A)': '3.75',
    }


def test_battery_state(wrapper):
    battery = wrapper.build_battery_state(12.3)
    assert battery.voltage == pytest.approx(12.3)
    assert battery.present
    assert math.isnan(battery.percentage)
    assert math.isnan(battery.current)
