import asyncio

import cake
from cake.modules.hardware.wheels.WheelsGazebo import WheelsGazebo
from geometry_msgs.msg import Twist


def test_init():
    props = {
        'name': 'name',
        'sim': True,
        'hardware': {
            'wheels-1': {
                'type': 'wheels',
            }
        }
    }
    robot = cake.Robot(props)
    assert isinstance(robot.wheels, WheelsGazebo)
    robot.wheels.set_speed(2)
    robot.shutdown()


def test_set_speed():
    props = {
        'name': 'name',
        'sim': True,
        'hardware': {
            'wheels-1': {
                'type': 'wheels',
            }
        }
    }
    robot = cake.Robot(props)
    @robot.runtime.run_in_event_loop
    async def body():
        def callback(msg: Twist):
            global posted_target_speed
            posted_target_speed = msg.linear.x
        robot.runtime.ros_interface.create_subscription(
            Twist, '/cmd_vel', callback, 10
        )
        await robot.wheels.set_speed(2)
        await asyncio.sleep(1e-3)
        robot.runtime.assert_ros_interface_ok()
        assert posted_target_speed == 2

    body()
    robot.shutdown()


def test_set_rotation_rate():
    props = {
        'name': 'name',
        'sim': True,
        'hardware': {
            'wheels-1': {
                'type': 'wheels',
            }
        }
    }
    robot = cake.Robot(props)
    @robot.runtime.run_in_event_loop
    async def body():
        def callback(msg: Twist):
            global posted_target_speed
            posted_target_speed = msg.angular.z
        robot.runtime.ros_interface.create_subscription(
            Twist, '/cmd_vel', callback, 10
        )
        await robot.wheels.set_rotation_rate(3)
        await asyncio.sleep(1e-3)
        robot.runtime.assert_ros_interface_ok()
        assert posted_target_speed == 3

    body()
    robot.shutdown()
