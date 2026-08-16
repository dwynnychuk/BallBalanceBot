import pytest
from unittest.mock import MagicMock
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
    assert result == 60
    assert servo.currentAngle == 60

def test_rotate_absolute_below_range():
    servo = Servo(0, kit=None)
    
    result = servo.rotate_absolute(90)
    assert result == servo.MIN_ANGLE

def test_rotate_absolute_above_range():
    servo = Servo(0, kit=None)
    
    result = servo.rotate_absolute(-50)
    assert result == servo.MAX_ANGLE
    
def test_rotate_absolute_sets_kit_angle():
    mock_kit = MagicMock()
    servo = Servo(0, kit=mock_kit)
    
    result = servo.rotate_absolute(30)
    
    assert result == 60
    assert mock_kit.servo[0].angle == 60
    
def test_reset():
    mock_kit = MagicMock()
    servo = Servo(0, kit=mock_kit, home_angle=45)
    
    servo.reset()
    
    assert mock_kit.servo[0].angle == 45
    
def test_servo_hardware_error():
    with pytest.raises(RuntimeError):
        raise ServoHardwareError()
    
def test_initalize_servo_range():
    mock_hat = MagicMock()
    
    Servo._initialize_servo_range(mock_hat, 3, 500, 2500)
    
    mock_hat.servo[0].set_pulse_width_range.assert_called_with(500,2500)
    mock_hat.servo[2].set_pulse_width_range.assert_called_with(500,2500)