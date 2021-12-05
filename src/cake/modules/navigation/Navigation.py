import cake.runtime.runtime as runtime
from cake.exceptions import UndefinedHardware, Unimplemented

class Navigation:
    def __init__(self, robot):
        self.robot = robot
        self.initialized = False


    def init(self, props):
        from .from_props import from_props
        if self.initialized:
            raise Exception("Module already initialized.")
        new_self = from_props(props, self.robot)
        if new_self is not None:
            self.robot.navigation = new_self
            self.initialized = True


    @runtime.run_in_event_loop
    async def explore(self, method='random_walk', timeout=None):
        """
        Explores the room and generates a map using SLAM.

        Parameters
        ----------
        method
            Exploration method: only 'random_walk' for now.

        timeout
            Duration of exploration in seconds.
        """
        self._raise_undefined_or_unimplemented()


    @runtime.run_in_event_loop
    async def move_to(self, target_x, target_y, target_heading=None):
        """
        Moves the robot to the target position.

        Parameters
        ----------
        target_x
            X-location of the target.
            If the robot is performing in SLAM mode, origin will be the
            starting point and x will correspond to right direction at starting moment.

        target_y
            y-location of the target.
            If the robot is performing in SLAM mode, origin will be the
            starting point and y will correspond to forward direction at starting moment.

        target_heading
            Target heading in radians. Optional.
        """
        self._raise_undefined_or_unimplemented()


    def _raise_undefined_or_unimplemented(self):
        if self.initialized:
            raise Unimplemented
        raise UndefinedHardware