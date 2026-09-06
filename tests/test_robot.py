import pytest

from robot import LinkLengths, Robot


def test_link_lengths_default():
    l = LinkLengths()
    
    assert l.L0 == 0.105
    assert l.L1 == 0.08
    assert l.L2 == 0.046
    assert l.L3 == 0.0935
    
def test_link_lengths_rejects_zero():
    with pytest.raises(ValueError):
        LinkLengths(L0=0.0)

def test_link_lengths_rejects_negative():
    with pytest.raises(ValueError):
        LinkLengths(L3=-0.1)
        
def test_link_lengths_custom():
    l = LinkLengths(L0=0.1, L1=0.2, L2=0.3, L3=0.4)
    
    assert l.L0 == 0.1
    assert l.L1 == 0.2
    assert l.L2 == 0.3
    assert l.L3 == 0.4
    
def test_robot_link_lengths_default():
    robot = Robot()
    assert robot.links == LinkLengths()

def test_robot_link_lengths_custom():
    custom = LinkLengths(L0 = 0.1, L1 = 0.2, L2 = 0.3, L3 = 0.4)
    robot = Robot(links=custom)
    
    assert robot.links == custom
    
def test_robot_init_none():
    links = None
    robot = Robot(links=links)
    
    assert robot.links == LinkLengths()
    
def test_validate_ik_inputs_nominal():
    robot = Robot()
    robot._validate_ik_inputs([0.1, 0.1, 0.1], 0.25)
    
def test_validate_ik_inputs_length_not_three():
    robot = Robot()
    with pytest.raises(ValueError):
        robot._validate_ik_inputs([0.1, 0.2, 0.3, 0.4], 0.25)

def test_validate_ik_inputs_height_zero():
    robot = Robot()
    with pytest.raises(ValueError):
        robot._validate_ik_inputs([0.1, 0.2, 0.3], 0)

def test_validate_ik_inputs_height_negative():
    robot = Robot()
    with pytest.raises(ValueError):
        robot._validate_ik_inputs([0.1, 0.2, 0.3, 0.4], -0.25)

def test_validate_ik_inputs_no_height():
    robot = Robot()
    with pytest.raises(TypeError):
        robot._validate_ik_inputs([0.1, 0.2, 0.3])

def test_validate_ik_inputs_no_normal():
    robot = Robot()
    with pytest.raises(TypeError):
        robot._validate_ik_inputs(height=0.25)
        
def test_check_discriminant_no_discriminant():
    robot = Robot()
    with pytest.raises(TypeError):
        robot._check_discriminant()
        
def test_check_discriminant_lessthan_zero():
    robot = Robot()
    with pytest.raises(ValueError):
        robot._check_discriminant(-0.1)
        
def test_check_discriminant_is_zero():
    robot = Robot()
    robot._check_discriminant(0.0)
        
def test_check_discriminant_nominal():
    robot = Robot()
    robot._check_discriminant(0.1)
    
def test_arm_angles_equal_for_flat_plate():
    robot = Robot()
    normal = [0.0, 0.0, 1.0]
    height = 0.15
    ref_h = robot._compute_reference_height(height)

    theta1 = robot._compute_arm_angle_1(normal, height, ref_h)
    theta2 = robot._compute_arm_angle_2(normal, height, ref_h)
    theta3 = robot._compute_arm_angle_3(normal, height, ref_h)

    assert theta1 == pytest.approx(theta2)
    assert theta2 == pytest.approx(theta3)
    
def test_compute_reference_height_baseline():
    robot = Robot()
    height = 0.15
    ref_h = robot._compute_reference_height(height)
    
    assert ref_h == pytest.approx(0.0897458905)
    
def test_compute_unreachable_reference_height():
    robot = Robot()
    height = 0.2

    with pytest.raises(ValueError):
        robot._compute_reference_height(height=height)