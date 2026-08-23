from unittest.mock import MagicMock, patch

import pytest

from classServo import Servo, ServoGroup, ServoHardwareError, init_servos


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
    
def test_servo_group_home_all_updates_current_angle():
    home_angle = 15
    servo_0 = Servo(0, kit=None, home_angle=home_angle)
    servo_1 = Servo(1, kit=None, home_angle=home_angle)
    
    servo_group = ServoGroup([servo_0, servo_1])
    servo_group.home_all()
    
    assert servo_0.currentAngle == home_angle
    
def test_servo_group_home_all_updates_kit():
    servo_0 = MagicMock()
    servo_1 = MagicMock()
    
    servo_group = ServoGroup([servo_0, servo_1])
    servo_group.home_all()
    
    servo_0.reset.assert_called_once()
    servo_1.reset.assert_called_once()
    
def test_servo_group_rotate_all():
    servo_0 = MagicMock()
    servo_1 = MagicMock()
    
    servo_group = ServoGroup([servo_0, servo_1])
    servo_group.rotate_all(30)
    
    servo_0.rotate_absolute.assert_called_once_with(30)
    
def test_servo_group_apply_angles_args_num():
    servo_0 = MagicMock()
    servo_1 = MagicMock()
    
    servo_group = ServoGroup([servo_0, servo_1])
    
    with pytest.raises(ValueError):
        servo_group.apply_angles([10])
    
    with pytest.raises(ValueError):
        servo_group.apply_angles([10,20,30])
        
def test_servo_group_apply_angles_calls_each_servo():
    servo_0 = MagicMock()
    servo_1 = MagicMock()
    
    servo_group = ServoGroup([servo_0, servo_1])
    servo_group.apply_angles([10,20])
    
    servo_0.rotate_absolute.assert_called_once_with(10)
    servo_1.rotate_absolute.assert_called_once_with(20)
    
def test_servo_group_apply_angles_returns():
    servo_0 = MagicMock()
    servo_1 = MagicMock()
    
    servo_0.rotate_absolute.return_value = 60
    servo_1.rotate_absolute.return_value = 70
    
    servo_group = ServoGroup([servo_0, servo_1])
    result = servo_group.apply_angles([30,20])
    
    assert result == [60,70]
    
def test_init_servos_no_pi():
    with patch("classServo.is_pi", False):
        group = init_servos(3)
        
    assert isinstance(group, ServoGroup)
    assert len(group.servos) == 3
    for servo in group.servos:
        assert servo.kit is None
        assert servo.currentAngle == servo.home_angle