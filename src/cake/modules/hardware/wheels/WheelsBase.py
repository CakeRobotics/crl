import asyncio

from cake.runtime.runtime import run_in_event_loop
from geometry_msgs.msg import Twist, Vector3
import rclpy.qos
from .Wheels import Wheels

class WheelsBase(Wheels):
    def __init__(self, robot, _props):
        self.robot = robot
        self._set_initial_values()
        self._init_topic_handles()
        self._start_publish_loop()

    def _set_initial_values(self):
        self._target_speed = 0
        self._target_rotation_rate = 0

    async def shutdown(self):
        await self.stop()

    @run_in_event_loop
    async def _init_topic_handles(self):
        self._velocity_publisher = self.robot.runtime.ros_interface.create_publisher(
            Twist, '/cmd_vel', rclpy.qos.HistoryPolicy.KEEP_LAST
        )

    @run_in_event_loop
    async def set_speed(self, _target_speed):
        if self.robot.navigation.initialized:
            await self.robot.navigation.cancel_task()
        self._target_speed = _target_speed
        self._publish_velocity_command()

    @run_in_event_loop
    async def set_rotation_rate(self, _target_rotation_rate):
        if self.robot.navigation.initialized:
            await self.robot.navigation.cancel_task()
        self._target_rotation_rate = _target_rotation_rate
        self._publish_velocity_command()

    @run_in_event_loop
    async def stop(self):
        await self.set_speed(0.0)
        await self.set_rotation_rate(0.0)

    def _start_publish_loop(self):
        async def runner():
            while True:
                self._publish_velocity_command()
                await asyncio.sleep(0.01)
        self.robot.runtime.start_task(runner())

    def _publish_velocity_command(self):
        msg = Twist()
        msg.linear = Vector3(x=float(self._target_speed))
        msg.angular = Vector3(z=float(self._target_rotation_rate))
        self._velocity_publisher.publish(msg)
