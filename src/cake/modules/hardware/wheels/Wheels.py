import cake.runtime.runtime as runtime
from cake.exceptions import UndefinedHardware, Unimplemented

class Wheels:
    def __init__(self, robot):
        self.robot = robot
        self.initialized = False

    def init(self, props):
        from .from_props import from_props
        if self.initialized:
            raise Exception("Module already initialized.")
        new_self = from_props(props, self.robot)
        if new_self is not None:
            self.robot.wheels = new_self
            self.initialized = True

    @runtime.run_in_event_loop
    async def set_speed(self, target_speed):
        self._raise_undefined_or_unimplemented()

    @runtime.run_in_event_loop
    async def set_rotation_rate(self, target_rotation_rate):
        self._raise_undefined_or_unimplemented()


    def _raise_undefined_or_unimplemented(self):
        if self.initialized:
            raise Unimplemented
        raise UndefinedHardware
