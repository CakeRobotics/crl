from asyncio import sleep
import asyncio
from datetime import datetime
import logging
from random import random

import numpy as np
from numpy.linalg import norm
import rclpy.qos
from nav_msgs.msg._occupancy_grid import OccupancyGrid
from visualization_msgs.msg._marker import Marker
from geometry_msgs.msg._pose import Pose
import skimage.measure
import skimage.morphology
import scipy.ndimage

from cake import Robot
from cake.utils.clamped_sleep import clamped_sleep

# Doesn't hit the walls, doesn't revisit known places, continues until
# the entire closed environment is mapped.
#
# Not suitable for open environments.
#

class FullScanAgent:
    def __init__(self, robot: Robot, timeout):
        self.robot = robot
        self.timeout = timeout
        self.backoff_dist = 2.5
        self.lookahead_radius = 2 * self.backoff_dist
        self.init_knowledge_boundary_publisher()
        self.init_minxy_publisher()


    async def run(self):
        self.store_start_timestamp()
        await self.create_map_subscription()
        while not (await self.fully_mapped()) and not self.is_timeout_over():
            if self.any_walls_detected():
                await self.push_knowledge_boundary()
                # await self.random_step()
            else:
                await self.go_forward()
            await asyncio.sleep(0.1)


    async def random_step(self):
        logging.debug('Taking a random step.')
        robot_x, robot_y, _ = await self.robot.navigation.get_position()
        h = await self.robot.navigation.get_heading()
        a = h + 2 * 3.14 * random()
        r = np.asarray([robot_x, robot_y])
        step_size = 40
        step = step_size * np.asarray([np.cos(a), np.sin(a)])
        target = r + step
        await self.robot.navigation.move_to(target[0], target[1])


    def store_start_timestamp(self):
        self.start_timestamp = datetime.now().timestamp()


    async def create_map_subscription(self):
        logging.debug('[full_scan] Creating map subscription...')
        self.map = None
        self.callback_called_flag = False
        def map_update_callback(new_map):
            self.map_msg = new_map
            self.callback_called_flag = True
            if not new_map.data:
                return
            self.map_info = new_map.info
            self.map = np.asarray(new_map.data)
            self.map = self.map.reshape((new_map.info.height, -1))
        self.subscription = self.robot.runtime.ros_interface.create_subscription(
            OccupancyGrid,
            '/map',
            map_update_callback,
            rclpy.qos.HistoryPolicy.KEEP_LAST,
        )
        logging.debug('Waiting for map publisher...')
        while not self.callback_called_flag:
            await sleep(0.5)
        logging.debug('Map received.')
        # map: -1 = unknown, 0 = ground, 1 = wall


    async def fully_mapped(self):
        if self.map is None:
            return False
        # whether or not the robot is inside enclosed walls
        walls = np.where(self.map > 0, 1, 0)
        walls_expanded = skimage.morphology.dilation(walls)
        closed_loops = skimage.measure.find_contours(walls_expanded)
        if len(closed_loops) == 0:
            return False
        robot_x, robot_y, _ = await self.robot.navigation.get_position()
        i, j = self._xy2ij([robot_x, robot_y])
        for polygon in closed_loops:
            if skimage.measure.points_in_poly([(i, j)], polygon):
                return True
        return False


    def is_timeout_over(self):
        return datetime.now().timestamp() > self.start_timestamp + self.timeout


    def any_walls_detected(self):
        if self.map is None:
            return False
        return np.any(self.map > 0)


    async def move_near_closest_wall(self):
        walls_dist_to_robot = await self.get_wall_distmap()
        if np.nanmin(walls_dist_to_robot) < 2 * self.backoff_dist:
            logging.debug('[full_scan] Skipped moving near the closest wall as the robot is already too close.')
            return
        logging.debug('[full_scan] Moving near the closest wall...')
        wall = self.minimum_xy(walls_dist_to_robot)  # closest wall
        robot_x, robot_y, _ = await self.robot.navigation.get_position()
        me = np.asarray([robot_x, robot_y])
        target = wall + self.backoff_dist * (me - wall) / norm(me - wall)
        await self.robot.navigation.move_to(target[0], target[1])


    async def follow_the_wall(self):
        logging.debug('[full_scan] Following the wall...')
        lookahead_cost = await self.get_lookahead_cost()
        robot_x, robot_y, _ = await self.robot.navigation.get_position()
        wall = self.minimum_xy(lookahead_cost)  # target wall
        me = np.asarray([robot_x, robot_y])
        R90 = np.asarray([
            [0, 1],
            [-1, 0]
        ])
        target = wall + self.backoff_dist * R90 @ (me - wall) / norm(me - wall)
        await self.robot.navigation.move_to(target[0], target[1], wait_to_finish=False)


    def minimum_xy(self, map):
        closest_wall_to_robot_flatten_index = np.nanargmin(map)
        closest_wall_to_robot_ij = np.unravel_index(closest_wall_to_robot_flatten_index, map.shape)
        closest_wall_to_robot_xy = self._ij2xy(closest_wall_to_robot_ij)
        y, x = closest_wall_to_robot_xy  # :/
        self.publish_xy_marker(x, y)
        return np.asarray([x, y])


    async def go_forward(self):
        logging.debug('[full_scan] Going forward...')
        await self.robot.wheels.set_speed(0.3)


    def get_meshgrid(self):
        dx = dy = self.map_info.resolution
        w = self.map_info.width
        h = self.map_info.height
        x = self.map_info.origin.position.x + dx * np.arange(0, w)
        y = self.map_info.origin.position.y + dy * np.arange(0, h)
        X, Y = np.meshgrid(x, y)
        return X, Y


    async def get_wall_distmap(self):
        X, Y = self.get_meshgrid()
        robot_x, robot_y, _ = await self.robot.navigation.get_position()
        dist_to_robot = np.sqrt( (X - robot_x) ** 2 + (Y - robot_y) ** 2 )
        walls_dist_to_robot = np.where(self.map > 0, dist_to_robot, np.NaN)
        return walls_dist_to_robot

    async def get_distmap(self):
        X, Y = self.get_meshgrid()
        robot_x, robot_y, _ = await self.robot.navigation.get_position()
        dist_to_robot = np.sqrt( (X - robot_x) ** 2 + (Y - robot_y) ** 2 )
        return dist_to_robot

    def knowledge_boundaries(self):
        map = self.map
        map = np.array(map, dtype=float)
        smooth = skimage.morphology.dilation(map, footprint=np.ones((5, 5)))
        smooth[smooth == 100] = np.nan
        kernel = np.asarray([
            [-1, -1, -1],
            [-1, 8, -1],
            [-1, -1, -1]
        ])
        borders = scipy.ndimage.convolve(smooth, kernel)
        # borders[borders == np.nan] = 0
        borders = np.nan_to_num(borders)
        borders[borders != 0] = 1
        return borders


    def init_knowledge_boundary_publisher(self):
        self.knowledge_boundary_publisher = self.robot.runtime.ros_interface.create_publisher(
            OccupancyGrid, '/knowledge_boundaries', rclpy.qos.HistoryPolicy.KEEP_LAST
        )


    def init_minxy_publisher(self):
        self.minxy_publisher = self.robot.runtime.ros_interface.create_publisher(
            Marker, '/target', rclpy.qos.HistoryPolicy.KEEP_LAST
        )


    def publish_xy_marker(self, x, y):
        msg = Marker()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.type = 2
        msg.scale.x = 0.1
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        msg.color.a = 1.0
        msg.color.r = 1.0
        msg.header.frame_id = 'map'
        self.minxy_publisher.publish(msg)


    def visualize_knowledge_boundaries(self, boundaries):
        msg = self.map_msg
        msg.data = boundaries.flatten().astype(np.int).tolist()
        self.knowledge_boundary_publisher.publish(msg)


    def random_point_on_region(self, labels, index):
        region_size = scipy.ndimage.sum(1, labels, index)
        choice = np.random.randint(0, region_size)
        count = 0
        for i in range(labels.shape[0]):
            for j in range(labels.shape[1]):
                if labels[i, j] == index:
                    count += 1
                    if count == choice:
                        return self._ij2xy((i, j))


    def center_of_biggest_knowledge_boundary(self):
        _knowledge_boundaries = self.knowledge_boundaries()
        labels, N = scipy.ndimage.label(_knowledge_boundaries, np.ones((3, 3)))
        border_sizes = scipy.ndimage.sum(_knowledge_boundaries, labels, range(0, N+1))
        print(border_sizes)
        biggest_border_label = np.argmax(border_sizes)
        biggest_border_centroid = scipy.ndimage.center_of_mass(_knowledge_boundaries, labels, biggest_border_label)
        biggest_border_centroid_xy = self._ij2xy(biggest_border_centroid)
        return biggest_border_centroid_xy


    async def center_of_closest_knowledge_boundary(self):
        _knowledge_boundaries = self.knowledge_boundaries()
        labels, N = scipy.ndimage.label(_knowledge_boundaries, np.ones((3, 3)))
        robot_x, robot_y, _ = await self.robot.navigation.get_position()
        me = np.asarray([robot_x, robot_y])
        border_centroids = scipy.ndimage.center_of_mass(_knowledge_boundaries, labels, range(1, N+1))
        # biggest_border_centroid = scipy.ndimage.center_of_mass(_knowledge_boundaries, labels, biggest_border_label)
        # biggest_border_centroid_xy = self._ij2xy(biggest_border_centroid)
        centroid_distances = norm(np.subtract(border_centroids, me), axis=1)
        closest_centroid_id = np.argmin(centroid_distances)
        closest_centroid = border_centroids[closest_centroid_id]
        return self._ij2xy(closest_centroid)


    async def random_point_closest_knowledge_boundary(self):
        _knowledge_boundaries = self.knowledge_boundaries()
        labels, N = scipy.ndimage.label(_knowledge_boundaries, np.ones((3, 3)))
        robot_x, robot_y, _ = await self.robot.navigation.get_position()
        me = np.asarray([robot_x, robot_y])
        border_centroids = scipy.ndimage.center_of_mass(_knowledge_boundaries, labels, range(1, N+1))
        # biggest_border_centroid = scipy.ndimage.center_of_mass(_knowledge_boundaries, labels, biggest_border_label)
        # biggest_border_centroid_xy = self._ij2xy(biggest_border_centroid)
        centroid_distances = norm(np.subtract(border_centroids, me), axis=1)
        closest_centroid_id = np.argmin(centroid_distances)
        closest_region_label = closest_centroid_id + 1
        return self.random_point_on_region(labels, closest_region_label)


    async def select_point_on_knowledge_boundary(self):
        _knowledge_boundaries = self.knowledge_boundaries()
        walls = np.where(self.map > 0, 1, 0)
        wall_dilation_size = int(0.2 * self.backoff_dist / self.map_info.resolution)
        walls_dilated = skimage.morphology.dilation(walls, footprint=np.ones((wall_dilation_size, wall_dilation_size)))
        dist_map = await self.get_distmap()
        # dist_map[dist_map < self.backoff_dist] = np.nan  # flunk close boundaries
        self.visualize_knowledge_boundaries(np.nan_to_num(dist_map))
        cost = dist_map[:]
        cost[_knowledge_boundaries==0] = np.nan # Only keep knowledge boundaries
        cost[walls_dilated > 0] = np.nan # Only keep points far from the walls
        goal = self.minimum_xy(cost)
        return goal
        # labels, N = scipy.ndimage.label(_knowledge_boundaries, np.ones((3, 3)))
        # robot_x, robot_y, _ = await self.robot.navigation.get_position()
        # me = np.asarray([robot_x, robot_y])
        # border_centroids = scipy.ndimage.center_of_mass(_knowledge_boundaries, labels, range(1, N+1))
        # # biggest_border_centroid = scipy.ndimage.center_of_mass(_knowledge_boundaries, labels, biggest_border_label)
        # # biggest_border_centroid_xy = self._ij2xy(biggest_border_centroid)
        # centroid_distances = norm(np.subtract(border_centroids, me), axis=1)
        # closest_centroid_id = np.argmin(centroid_distances)
        # closest_region_label = closest_centroid_id + 1
        # return self.random_point_on_region(labels, closest_region_label)


    async def push_knowledge_boundary(self):
        logging.debug('Pushing a boundary...')
        boundary = await self.select_point_on_knowledge_boundary()
        # robot_x, robot_y, _ = await self.robot.navigation.get_position()
        # me = np.asarray([robot_x, robot_y])
        # target = boundary + self.backoff_dist * (me - boundary) / norm(me - boundary)
        # await self.robot.navigation.move_to(target[0], target[1])
        await self.robot.navigation.move_to(boundary[0], boundary[1])


    async def get_lookahead_cost(self):
        # C: lookahead cost
        # p: point location (independent variable)
        #
        # R: lookahead radius
        # r: robot position
        # a: robot heading unit vector
        #
        # Heuristic formula:
        # C(p) = (|p - r| - R) ^ 2 + 0.1 * ( (p - r) x a ) ^ 2
        # The first term introduces a lookahead distance
        # The second term adds a counter-clockwise preference

        px, py = self.get_meshgrid()
        robot_x, robot_y, _ = await self.robot.navigation.get_position()
        robot_heading = await self.robot.navigation.get_heading()

        a = np.asarray([np.cos(robot_heading), np.sin(robot_heading)])
        rp = np.asarray([px - robot_x, py - robot_y])
        R = self.lookahead_radius

        C = (norm(rp, axis=0) - R) ** 2 + 0.1 * np.cross(rp, a, axis=0) ** 2
        return C


    def _ij2xy(self, ij):
        dx = dy = self.map_info.resolution
        i, j = ij
        x = self.map_info.origin.position.x + dx * i
        y = self.map_info.origin.position.y + dy * j
        return x, y


    def _xy2ij(self, xy):
        dx = dy = self.map_info.resolution
        x, y = xy
        i = (x - self.map_info.origin.position.x) / dx
        j = (y - self.map_info.origin.position.y) / dy
        return i, j
