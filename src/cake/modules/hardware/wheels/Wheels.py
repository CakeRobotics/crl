from cake.runtime.runtime import run_in_event_loop
from cake.exceptions import UndefinedHardware
from cake.utils.filter_by_type import filter_by_type
from .WheelsBase import WheelsBase
from .WheelsDummy import WheelsDummy
from .WheelsGazebo import WheelsGazebo

class Wheels(WheelsBase):
    def __init__(self, robot):
        self.robot = robot

    def init(self, props):
        self.robot.wheels = self._from_props(props)


    def _from_props(self, props):
        hardware = props.get('hardware')
        if hardware is None:
            return self

        wheels_specs = filter_by_type(hardware, 'wheels')
        if len(wheels_specs) == 0:
            return self
        if len(wheels_specs) > 1:
            raise Exception("Only 1 set of wheels is currently supported.")

        specs = list(wheels_specs.items())[0][1]  # Specs of first wheels object
        if specs.get('dummy') == True:
            return WheelsDummy(self.robot)
        if specs.get('sim') == True:
            return WheelsGazebo(self.robot, props)


    @run_in_event_loop
    async def set_speed(self, _target_speed):
        raise UndefinedHardware

    @run_in_event_loop
    async def set_rotation_rate(self, _target_rotation_rate):
        raise UndefinedHardware
