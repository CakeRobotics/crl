from cake.runtime.runtime import run_in_event_loop
from geometry_msgs.msg import Twist, Vector3
import rclpy.qos
from .WheelsBase import WheelsBase

class WheelsGazebo(WheelsBase):
    def __init__(self, robot, _props):
        self.robot = robot
        self._set_initial_values()
        self._init_topic_handles()  # type: ignore

    def _set_initial_values(self):
        self._target_speed = 0
        self._target_rotation_rate = 0

    @run_in_event_loop
    async def _init_topic_handles(self):
        self._velocity_publisher = self.robot.runtime.ros_interface.create_publisher(
            Twist, '/cake/wheels/vel_cmd', rclpy.qos.HistoryPolicy.KEEP_LAST
        )

    @run_in_event_loop
    async def set_speed(self, _target_speed):
        self._target_speed = _target_speed
        self._publish_velocity_command()

    @run_in_event_loop
    async def set_rotation_rate(self, _target_rotation_rate):
        self._target_rotation_rate = _target_rotation_rate
        self._publish_velocity_command()

    def _publish_velocity_command(self):
        msg = Twist()
        msg.linear = Vector3(x=float(self._target_speed))
        msg.angular = Vector3(z=float(self._target_rotation_rate))
        self._velocity_publisher.publish(msg)
