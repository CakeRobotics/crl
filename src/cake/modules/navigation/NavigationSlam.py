import asyncio
import logging

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header
from transforms3d.euler import euler2quat, quat2euler
from nav2_simple_commander.robot_navigator import BasicNavigator

from cake.exceptions import Unimplemented
from cake.runtime.runtime import run_in_event_loop
from .external_nodes import generate_launch_description
from .Navigation import Navigation
from .explore.random_walk import random_walk


class NavigationSlam(Navigation):
    def __init__(self, robot, props):
        self.robot = robot
        self._init_nodes(props)
        self._init_navigator()

    @run_in_event_loop
    async def _init_navigator(self):
        self.navigator = BasicNavigator()
        self.navigator.get_logger().set_level(logging.WARNING)
        initial_pose = PoseStamped()
        self.navigator.setInitialPose(initial_pose)

    async def shutdown(self):
        # Upstream improvement proposed: https://github.com/ros-planning/navigation2/pull/2924
        self.navigator.nav_through_poses_client.destroy()
        self.navigator.nav_to_pose_client.destroy()
        self.navigator.follow_waypoints_client.destroy()
        self.navigator.compute_path_to_pose_client.destroy()
        self.navigator.compute_path_through_poses_client.destroy()
        self.navigator.destroy_node()

    @run_in_event_loop
    async def _init_nodes(self, props):
        launch_description = generate_launch_description(props)
        self.robot.runtime.ros_interface.launch_external_nodes(launch_description)
        self.robot.runtime.ros_interface.wait_for_node_to_activate('bt_navigator')

    @run_in_event_loop
    async def move_to(self, target_x, target_y, target_heading=None, wait_to_finish=True):
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
        # Block
        if not wait_to_finish:
            return
        while not self.navigator.isNavComplete(): # Will change to isTaskComplete
            await asyncio.sleep(0.1)

    @run_in_event_loop
    async def is_task_complete(self):
        return self.navigator.isNavComplete()

    @run_in_event_loop
    async def cancel_task(self):
        self.navigator.cancelNav()

    @run_in_event_loop
    async def stop(self):
        self.navigator.cancelNav()
        if self.robot.wheels.initialized:
            await self.robot.wheels.set_speed(0)
            await self.robot.wheels.set_rotation_rate(0)

    @run_in_event_loop
    async def get_position(self):
        # P = self.navigator.getFeedback().current_pose.pose.position
        transform_stamped = self.robot.runtime.ros_interface.lookup_transform('base_link', 'map')
        P = transform_stamped.transform.translation
        return P.x, P.y, P.z

    @run_in_event_loop
    async def get_heading(self):
        # Q = self.navigator.getFeedback().current_pose.pose.orientation
        transform_stamped = self.robot.runtime.ros_interface.lookup_transform('base_link', 'map')
        Q = transform_stamped.transform.rotation
        heading, _, _ = quat2euler((Q.x, Q.y, Q.z, Q.w), 'sxyz')  # FIXME; Unstable
        return heading

    @run_in_event_loop
    async def explore(self, method='random_walk', timeout=None):
        if method == 'random_walk':
            await random_walk(self.robot, timeout=timeout)
        else:
            raise Unimplemented()
