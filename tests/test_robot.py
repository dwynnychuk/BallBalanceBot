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
        robot._validate_ik_inputs([0.1, 0.2, 0.3, 0.4], 0)

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