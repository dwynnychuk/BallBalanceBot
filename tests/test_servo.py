import pytest
from classServo import Servo, ServoHardwareError

def test_servo_init_defaults():
    servo = Servo(2, kit=None)
    
    assert servo.id == 2
    assert servo.kit == None
    assert servo.home_angle == 90
    assert servo.currentAngle == None
    
def test_servo_init_with_home_angle():
    servo = Servo(2, kit=None, home_angle=45)
    
    assert servo.home_angle == 45

def test_run_position_offset():
    servo = Servo(0, kit=None)
    
    assert servo._run_position_offset(0) == 90
    assert servo._run_position_offset(90) == 0
    assert servo._run_position_offset(30) == 60
    
def test_rotate_absolute_in_range():
    servo = Servo(0, kit=None)
    
    result = servo.rotate_absolute(30)
    pass

def test_rotate_absolute_below_range():
    pass

def test_rotate_absolute_above_range():
    pass

def test_servo_hardware_error():
    with pytest.raises(RuntimeError):
        raise ServoHardwareError()