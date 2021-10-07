import cake
import pytest

def test_empty_props():
    robot = cake.Robot({})
    assert robot.health
    robot.shutdown()

def test_auto_props():
    with pytest.raises(Exception, match=r"Can't find `.+\.yaml` or `.+\.json`"):
        _robot = cake.Robot()
