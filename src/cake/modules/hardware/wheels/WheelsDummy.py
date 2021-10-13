from cake.runtime.runtime import run_in_event_loop
from .Wheels import Wheels

class WheelsDummy(Wheels):
    def __init__(self, robot):
        self.robot = robot

    @run_in_event_loop
    async def set_speed(self, _target_speed):
        pass
