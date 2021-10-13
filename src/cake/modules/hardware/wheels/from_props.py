from cake.utils.filter_by_type import filter_by_type
from .WheelsDummy import WheelsDummy
from .WheelsGazebo import WheelsGazebo

def from_props(props, robot):
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
