from nav_msgs.msg._occupancy_grid import OccupancyGrid
import numpy as np
import rclpy.qos
from visualization_msgs.msg._marker import Marker


class MapUtils:
    def __init__(self, robot, topic='/map'):
        self.robot = robot
        self._map_publishers = {}
        self._point_publishers = {}
        self._map_ready = False
        self.topic = topic

    async def start(self):
        await self._create_map_subscription(self.topic)

    async def _create_map_subscription(self, topic):
        def map_update_callback(new_map):
            self._map_msg = new_map
            if not new_map.data:
                return
            self._map_ready = True
            self._map_info = new_map.info
            self._map = np.asarray(new_map.data)
            self._map = self._map.reshape((new_map.info.height, -1))
        self.subscription = self.robot.runtime.ros_interface.create_subscription(
            OccupancyGrid,
            topic,
            map_update_callback,
            rclpy.qos.HistoryPolicy.KEEP_LAST,
        )
        # map: -1 = unknown, 0 = ground, 1 = wall

    def ready(self):
        return self._map_ready

    def get_map(self):
        assert self.ready(), "Map is not ready."
        return self._map

    def get_meshgrid(self):
        dx = dy = self._map_info.resolution
        w = self._map_info.width
        h = self._map_info.height
        x = self._map_info.origin.position.x + dx * np.arange(0, w)
        y = self._map_info.origin.position.y + dy * np.arange(0, h)
        X, Y = np.meshgrid(x, y)
        return X, Y

    async def get_distmap(self, ref_point):
        X, Y = self.get_meshgrid()
        dist = np.sqrt( (X - ref_point[0]) ** 2 + (Y - ref_point[1]) ** 2 )
        return dist

    def visualize_map(self, map, topic):
        msg = self._map_msg
        msg.data = map.flatten().astype(np.int).tolist()
        self.get_map_publisher(topic).publish(msg)

    def get_map_publisher(self, topic):
        publisher = self._map_publishers.get(topic)
        if publisher is None:
            publisher = self.robot.runtime.ros_interface.create_publisher(
                OccupancyGrid, topic, rclpy.qos.HistoryPolicy.KEEP_LAST
            )
            self._map_publishers[topic] = publisher
        return publisher

    def get_point_publisher(self, topic):
        publisher = self._point_publishers.get(topic)
        if publisher is None:
            publisher = self.robot.runtime.ros_interface.create_publisher(
                Marker, topic, rclpy.qos.HistoryPolicy.KEEP_LAST
            )
            self._point_publishers[topic] = publisher
        return publisher

    def visualize_point(self, point, topic):
        msg = Marker()
        msg.pose.position.x = point[0]
        msg.pose.position.y = point[1]
        msg.type = 2
        msg.scale.x = 0.1
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        msg.color.a = 1.0
        msg.color.r = 1.0
        msg.header.frame_id = 'map'
        self.get_point_publisher(topic).publish(msg)

    def ij2xy(self, ij):
        dx = dy = self._map_info.resolution
        i, j = ij
        x = self._map_info.origin.position.x + dx * i
        y = self._map_info.origin.position.y + dy * j
        return x, y


    def xy2ij(self, xy):
        dx = dy = self._map_info.resolution
        x, y = xy
        i = (x - self._map_info.origin.position.x) / dx
        j = (y - self._map_info.origin.position.y) / dy
        return i, j

    def is_point_inside_map(self, xy):
        i, j = self.xy2ij(xy)
        if i < 0 or i >= self._map_info.width or j < 0 or j >= self._map_info.height:
            return False
        return True

    def get_point_value(self, xy):
        assert self.is_point_inside_map(xy), 'Point is not inside the map.'
        i, j = self.xy2ij(xy)
        return self.get_map()[i, j]
