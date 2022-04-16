import rclpy.qos
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header
from transforms3d.euler import euler2quat

from cake.exceptions import Unimplemented
from cake.runtime.runtime import run_in_event_loop
from .external_nodes import generate_launch_description
from .Navigation import Navigation
from .explore.random_walk import random_walk


class NavigationSlam(Navigation):
    def __init__(self, robot, props):
        self.robot = robot
        self._set_initial_values()
        self._init_topic_handles()  # type: ignore
        self._init_nodes(props)  # type: ignore

    def _set_initial_values(self):
        self._target_position = (0.0, 0.0, 0.0)
        self._target_heading = 0.0

    @run_in_event_loop
    async def _init_topic_handles(self):
        self._goal_publisher = self.robot.runtime.ros_interface.create_publisher(
            PoseStamped, '/goal_pose', rclpy.qos.HistoryPolicy.KEEP_LAST
        )

    @run_in_event_loop
    async def _init_nodes(self, props):
        launch_description = generate_launch_description(props)
        self.robot.runtime.ros_interface.launch_external_nodes(launch_description)

    @run_in_event_loop
    async def move_to(self, target_x, target_y, target_heading=None):
        await self.robot.wheels.set_speed(0)
        await self.robot.wheels.set_rotation_rate(0)
        self._target_position = (float(target_x), float(target_y), 0.0)
        if target_heading is not None:
            self._target_heading = float(target_heading)
        self._publish_goal_command()

    @run_in_event_loop
    async def explore(self, method='random_walk', timeout=None):
        if method == 'random_walk':
            await random_walk(self.robot, timeout=timeout)
        else:
            raise Unimplemented()

    def _publish_goal_command(self):
        Px, Py, Pz = self._target_position
        # Qw, Qx, Qy, Qz = euler2quat(0, 0, self._target_heading, 'sxyz')
        Qw, Qx, Qy, Qz = euler2quat(0, self._target_heading, 0, 'sxyz')
        msg = PoseStamped(
            header=Header(
                stamp=Time(sec=1),
                frame_id='map',
            ),
            pose=Pose(
                position=Point(x=Px, y=Py, z=Pz),
                # orientation=Quaternion(x=Qx, y=Qy, z=Qz, w=Qw)
            )
        )
        self._goal_publisher.publish(msg)
