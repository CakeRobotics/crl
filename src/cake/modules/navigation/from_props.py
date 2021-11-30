from cake.utils.filter_by_type import filter_by_type
from .NavigationSlam import NavigationSlam

def from_props(props, robot):
    hardware = props.get('hardware')
    if hardware is None:
        return None

    wheels_specs = filter_by_type(hardware, 'wheels')
    if len(wheels_specs) == 0:
        return None

    imu_specs = filter_by_type(hardware, 'imu')
    if len(imu_specs) == 0:
        return None

    lidar_specs = filter_by_type(hardware, 'lidar')
    if len(lidar_specs) == 0:
        return None

    return NavigationSlam(robot, props)
