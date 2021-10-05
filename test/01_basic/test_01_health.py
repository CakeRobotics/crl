from cake import Robot
from time import sleep

def test_health():
    robot = Robot()
    assert robot.health()
    robot.shutdown()
