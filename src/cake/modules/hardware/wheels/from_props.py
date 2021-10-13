from cake.utils.filter_by_type import filter_by_type
from .WheelsDummy import WheelsDummy
from .WheelsGazebo import WheelsGazebo

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
            - dummy
            - boolean
            - false
            - If true, none of the function calls on this object will
              do anything. Also, all sensor-related calls will
              return fake data.
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
    if specs.get('dummy') == True:
        return WheelsDummy(robot)
    elif props.get('sim') == True:
        return WheelsGazebo(robot, props)
