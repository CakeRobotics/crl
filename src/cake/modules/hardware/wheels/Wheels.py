from cake.runtime.runtime import run_in_event_loop
from cake.exceptions import UndefinedHardware
from .WheelsBase import WheelsBase
from .WheelsDummy import WheelsDummy
from .WheelsGazebo import WheelsGazebo

class Wheels(WheelsBase):
    def __init__(self, robot):
        self.robot = robot

    def init(self, props):
        if len(props) == 0:
            return
        self._validate_props(props)
        self.robot.wheels = self._from_props(props)

    def _validate_props(self, props):
        number_of_specs = len(props)
        if number_of_specs > 1:
            raise Exception("Only 1 set of wheels is currently supported.")

    def _from_props(self, props):
        _name, specs = list(props.items())[0]
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
