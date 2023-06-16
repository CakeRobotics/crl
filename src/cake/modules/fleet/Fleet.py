import cake.runtime.runtime as runtime
from cake.exceptions import UndefinedHardware, Unimplemented

class Fleet:
    def __init__(self, robot):
        self.robot = robot
        self.initialized = False


    def init(self, props):
        from .from_props import from_props
        if self.initialized:
            raise Exception("Module already initialized.")
        new_self = from_props(props, self.robot)
        if new_self is not None:
            self.robot.fleet = new_self
            self.robot.fleet.initialized = True


    async def shutdown(self):
        pass


    @runtime.run_in_event_loop
    async def join(self, timeout=None):
        """
        Join the fleet and follow orders indefinitely. (Blocking)

        Parameters
        ----------
        timeout
            Duration of exploration in seconds.
        """
        self._raise_undefined_or_unimplemented()


    def _raise_undefined_or_unimplemented(self):
        if self.initialized:
            raise Unimplemented
        raise UndefinedHardware
