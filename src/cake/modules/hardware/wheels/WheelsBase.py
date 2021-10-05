from abc import ABC
from cake.exceptions import Unimplemented
from cake.runtime.runtime import run_in_event_loop

class WheelsBase(ABC):
    @run_in_event_loop
    async def set_speed(self, _target_speed):
        raise Unimplemented

    @run_in_event_loop
    async def set_rotation_rate(self, _target_rotation_rate):
        raise Unimplemented
