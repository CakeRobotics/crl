import pytest
import cake


def test_uninitialized():
    robot = cake.Robot({})
    with pytest.raises(cake.exceptions.UndefinedHardware):
        robot.wheels.set_speed(2)  # type: ignore
    robot.shutdown()


def test_initialized_without_wheels():
    props = {
        'name': 'name',
        'hardware': {}
    }
    robot = cake.Robot(props)
    with pytest.raises(cake.exceptions.UndefinedHardware):
        robot.wheels.set_speed(2)  # type: ignore
    robot.shutdown()


def test_initialized_with_too_many_wheels():
    props = {
        'name': 'name',
        'hardware': {
            'wheels-1': {
                'type': 'wheels',
            },
            'wheels-2': {
                'type': 'wheels',
            }
        }
    }
    with pytest.raises(Exception, match='Only 1 set of wheels is currently supported.'):
        cake.Robot(props)


def test_initialized():
    props = {
        'name': 'name',
        'hardware': {
            'wheels-1': {
                'type': 'wheels',
                'driver': 'dummy',
            }
        }
    }
    robot = cake.Robot(props)
    robot.wheels.set_speed(2)  # type: ignore
    robot.shutdown()
