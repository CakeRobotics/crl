import pytest

from cake import Robot
from std_msgs.msg import String


def test_thread_check_false():
    robot = Robot({})
    assert robot.health()
    check = robot.runtime.am_i_running_in_event_loop_thread()
    assert check == False
    robot.shutdown()

def test_thread_check_true():
    robot = Robot({})
    assert robot.health()
    @robot.runtime.run_in_event_loop
    async def body():
        return robot.runtime.am_i_running_in_event_loop_thread()
    check = body()
    assert check == True
    robot.shutdown()

def test_ros_interface_thread_restriction():
    robot = Robot({})
    assert robot.health()
    with pytest.raises(Exception, match='This function can only be called from event loop thread.'):
        robot.runtime.ros_interface.create_publisher(String, '/something', 10)
    robot.runtime.ros_interface.create_publisher(String, '/something', 10, thread_check=False)
    @robot.runtime.run_in_event_loop
    async def body():
        robot.runtime.ros_interface.create_publisher(String, '/something', 10)
    body()
    robot.shutdown()
