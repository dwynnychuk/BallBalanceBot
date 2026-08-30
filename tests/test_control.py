import time

import pytest

from control import PID, PIDGains, PIDState


def test_pid_init_defaults():
    pid = PID()
    assert pid.deadband == 40
    assert pid.max_integral == 800
    assert pid.prev_time is None
    
def test_pid_with_inputs():
    gains = PIDGains(kp=0.001,
                     ki=0.0002,
                     kd=0.00022)
    pid = PID(deadband=60,
              max_integral=600,
              gains=gains)
    assert pid.deadband == 60
    assert pid.max_integral == 600
    assert pid.gains.kp == 0.001
    assert pid.gains.ki == 0.0002
    assert pid.gains.kd == 0.00022
    
def test_clamp_nominal():
    assert PID()._clamp(5.0,1.0,10.0) == 5.0

def test_clamp_below():
    assert PID()._clamp(0.0, 1.0, 10.0) == 1.0

def test_clamp_above():
    assert PID()._clamp(11.0, 1.0, 10.0) == 10.0
    
def test_reset():
    pid = PID()
    
    pid.state_x.error = 0.1
    pid.state_y.error = 0.1
    pid.state_x.velocity = 0.25
    pid.state_y.velocity = 0.35
    pid.state_x.integral = 0.01
    pid.state_y.integral = 0.02
    pid.state_x.prev_measurement = 0.05
    pid.state_y.prev_measurement = 0.06
    
    pid.reset()
    
    assert pid.state_x == PIDState()
    assert pid.state_y == PIDState()
    assert pid.prev_time == None
    
def test_compute_axis_output_error_initial():
    gains = PIDGains(1,1,1)
    pid = PID(gains, 0.05, 800)
    state = PIDState()
    output = pid._compute_axis_output(1.0, 0.9, state, 0.01, "X")
    velocity = 0
    integral = 0.01 * 0.1
    expected_output = (1*0.1) + (1*integral) - (1*velocity)
    
    assert state.error == pytest.approx(0.1)
    assert state.prev_measurement == 0.9
    assert state.velocity == pytest.approx(velocity)
    assert state.integral == pytest.approx(integral)
    assert output == pytest.approx(expected_output)
    
def test_compute_axis_output_error_lessthan_deadband():
    gains = PIDGains(1,1,1)
    pid = PID(gains, 0.2, 800)
    state = PIDState(integral=100)
    output = pid._compute_axis_output(1.0, 0.9, state, 0.01, "X")
    velocity = 0
    integral = 95
    expected_output = (1*0) + (1*integral) - (1*velocity)
    
    assert state.error == pytest.approx(0.0)
    assert state.velocity == pytest.approx(velocity)
    assert state.integral == pytest.approx(integral)
    assert output == pytest.approx(expected_output)

def test_compute_axis_output_error_with_prev_measurement():
    gains = PIDGains(1,1,1)
    pid = PID(gains, 0.05, 800)
    state = PIDState(prev_measurement=0.8)
    output = pid._compute_axis_output(1.0, 0.9, state, 0.01, "X")
    velocity = (0.9-0.8)/0.01
    integral = 0.1 * 0.01
    expected_output = (1*0.1) + (1*integral) - (1*velocity)
    
    assert state.error == pytest.approx(0.1)
    assert state.velocity == pytest.approx(velocity)
    assert state.integral == pytest.approx(integral)
    assert output == pytest.approx(expected_output)
    
def test_initialize_state():
    pid = PID()
    pid.state_x = PIDState(prev_measurement=0.5)
    pid.state_y = PIDState(prev_measurement=0.4)
    now = time.perf_counter()
    
    pid._initialize_state(measurement=(0.3,0.3), current_time=now)

    assert pid.prev_time == pytest.approx(now)
    assert pid.state_x.prev_measurement == pytest.approx(0.3)
    assert pid.state_y.prev_measurement == pytest.approx(0.3)    
    
def test_get_current_output():
    gains = PIDGains(kp=1, ki=1, kd=1)
    state_x = PIDState(error=0.1, velocity=0.7, integral=1.1)
    state_y = PIDState(error=0.1, velocity=0.7, integral=1.1)
    
    pid = PID(gains=gains)
    pid.state_x = state_x
    pid.state_y = state_y
    output = pid._get_current_output()
    expected_output_x = -((1*0.1) + (1*1.1) - (1*0.7))
    expected_output_y = (1*0.1) + (1*1.1) - (1*0.7)
    
    assert output[0] == pytest.approx(-output[1])
    assert output[0] == pytest.approx(expected_output_x)
    assert output[1] == pytest.approx(expected_output_y)
    
def test_compute_output():
    pass
    