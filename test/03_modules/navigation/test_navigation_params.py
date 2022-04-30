import pytest
import cake


def test_uninitialized():
    robot = cake.Robot({})
    with pytest.raises(cake.exceptions.UndefinedHardware):
        robot.navigation.move_to(0, 0)
    robot.shutdown()


def test_initialized_without_wheels():
    props = {
        'name': 'name',
        'sim': True,
        'hardware': {
            'imu-1': {'type': 'imu'},
            'lidar-1': {'type': 'lidar'},
        }
    }
    robot = cake.Robot(props)
    with pytest.raises(cake.exceptions.UndefinedHardware):
        robot.navigation.move_to(0, 0)
    robot.shutdown()


def test_initialized_without_imu():
    props = {
        'name': 'name',
        'sim': True,
        'hardware': {
            'wheels-1': {'type': 'wheels'},
            'lidar-1': {'type': 'lidar'},
        }
    }
    robot = cake.Robot(props)
    with pytest.raises(cake.exceptions.UndefinedHardware):
        robot.navigation.move_to(0, 0)
    robot.shutdown()


def test_initialized_without_lidar():
    props = {
        'name': 'name',
        'sim': True,
        'hardware': {
            'wheels-1': {'type': 'wheels'},
            'imu-1': {'type': 'imu'},
        }
    }
    robot = cake.Robot(props)
    with pytest.raises(cake.exceptions.UndefinedHardware):
        robot.navigation.move_to(0, 0)
    robot.shutdown()


def test_initialized_with_all_parts():
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
    robot.navigation.explore(timeout=1)
    robot.navigation.move_to(0, 0)
    robot.shutdown()
