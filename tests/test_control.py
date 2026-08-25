from control import PID, PIDGains


def test_pid_init_defaults():
    pid = PID()
    assert pid.deadband == 40
    assert pid.max_integral == 800
    assert pid.prev_time is None
    
def test_pi_with_inputs():
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
    pid = PID()
    assert pid._clamp(5.0,1.0,10.0) == 5.0

def test_clamp_below():
    pid = PID()
    assert pid._clamp(0.0, 1.0, 10.0) == 1.0

def test_clamp_above():
    pid = PID()
    assert pid._clamp(11.0, 1.0, 10.0) == 10.0