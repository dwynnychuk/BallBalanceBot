import pytest

from robot import LinkLengths


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