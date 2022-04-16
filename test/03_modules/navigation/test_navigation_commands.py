import asyncio

import cake
from cake.modules.navigation.NavigationSlam import NavigationSlam


def test_init():
    props = {
        'name': 'name',
        'sim': True,
        'hardware': {
            'wheels-1': {'type': 'wheels'},
            'imu-1': {'type': 'imu'},
            'lidar-1': {'type': 'lidar'},
        }
    }
    robot = cake.Robot(props)
    assert isinstance(robot.navigation, NavigationSlam)
    robot.navigation.move_to(0, 0)  # type: ignore
    robot.shutdown()


def test_explore():
    props = {
        'name': 'name',
        'sim': True,
        'hardware': {
            'wheels-1': {'type': 'wheels'},
            'imu-1': {'type': 'imu'},
            'lidar-1': {'type': 'lidar'},
        }
    }
    robot = cake.Robot(props)
    @robot.runtime.run_in_event_loop
    async def body():
        assert robot.wheels._target_speed == 0
        robot.runtime.start_task(
            robot.navigation.explore(timeout=1)
        )
        await asyncio.sleep(0.5)
        assert robot.wheels._target_speed > 0
        await asyncio.sleep(1)
        assert robot.wheels._target_speed == 0

        robot.runtime.start_task(
            robot.navigation.explore(timeout=8, method='random_walk')
        )
        await asyncio.sleep(0.1)
        assert robot.wheels._target_speed == 0.2
        assert robot.wheels._target_rotation_rate != 0.0
        await asyncio.sleep(3.0)
        assert robot.wheels._target_speed == 0.2
        assert robot.wheels._target_rotation_rate == 0.0
        await asyncio.sleep(4.0)
        assert robot.wheels._target_speed == 0.2
        assert robot.wheels._target_rotation_rate != 0.0
        await asyncio.sleep(1.0)
        assert robot.wheels._target_speed == 0.0
        assert robot.wheels._target_rotation_rate == 0.0

    body()  # type: ignore
    robot.shutdown()
