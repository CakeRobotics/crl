from cake.utils.filter_by_type import filter_by_type
from .WheelsDummy import WheelsDummy
from .WheelsGazebo import WheelsGazebo
from .WheelsROS1 import WheelsROS1
from .WheelsROS2 import WheelsROS2

def from_props(props, robot):
    """
    .. list-table:: Global props read by robot.wheels
        :width: 90%
        :widths: 15 15 15 50
        :header-rows: 1

        *
            - Key
            - Type
            - Default
            - Description
        *
            - sim
            - boolean
            - false
            - If set to true, the simulation driver will be used.


    .. list-table:: Hardware props read by robot.wheels
        :width: 90%
        :widths: 15 15 15 50
        :header-rows: 1

        *
            - Key
            - Type
            - Default
            - Description
        *
            - driver
            - 'ros2' | 'ros1' | 'dummy' | None
            - None
            - Driver to use for the wheels. Required if sim != True.
        *
            - steering
            - 'ackermann' | 'diff'
            - *required*
            - The steering method of the wheels.
    """

    hardware = props.get('hardware')
    if hardware is None:
        return None

    wheels_specs = filter_by_type(hardware, 'wheels')
    if len(wheels_specs) == 0:
        return None
    if len(wheels_specs) > 1:
        raise Exception("Only 1 set of wheels is currently supported.")

    specs = list(wheels_specs.items())[0][1]  # Specs of first wheels object
    if props.get('sim') == True:
        return WheelsGazebo(robot, props)
    elif specs.get('driver') == 'dummy':
        return WheelsDummy(robot)
    elif specs.get('driver') == 'ros2':
        return WheelsROS2(robot, props)
    elif specs.get('driver') == 'ros1':
        if props.get('ros1_port') is None:
            raise Exception("Wheels driver set to 'ros1' requires global prop 'ros1_port' to be set.")
        return WheelsROS1(robot, props)
