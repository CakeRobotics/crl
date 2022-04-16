from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header
from transforms3d.euler import euler2quat
from nav2_simple_commander.robot_navigator import BasicNavigator

from cake.exceptions import Unimplemented
from cake.runtime.runtime import run_in_event_loop
from .external_nodes import generate_launch_description
from .Navigation import Navigation
from .explore.random_walk import random_walk


class NavigationSlam(Navigation):
    def __init__(self, robot, props):
        self.robot = robot
        self.navigator = BasicNavigator()
        self._init_nodes(props)  # type: ignore

    @run_in_event_loop
    async def _init_nodes(self, props):
        launch_description = generate_launch_description(props)
        self.robot.runtime.ros_interface.launch_external_nodes(launch_description)

    @run_in_event_loop
    async def move_to(self, target_x, target_y, target_heading=None):
        Qw, Qx, Qy, Qz = euler2quat(0.0, 0.0, float(target_heading or 0.0), 'sxyz')
        pose_stamped = PoseStamped(
            header=Header(
                stamp=Time(sec=1),
                frame_id='map',
            ),
            pose=Pose(
                position=Point(x=float(target_x), y=float(target_y), z=0.0),
                orientation=Quaternion(x=Qx, y=Qy, z=Qz, w=Qw)
            )
        )
        self.navigator.goToPose(pose_stamped)

    @run_in_event_loop
    async def explore(self, method='random_walk', timeout=None):
        if method == 'random_walk':
            await random_walk(self.robot, timeout=timeout)
        else:
            raise Unimplemented()
