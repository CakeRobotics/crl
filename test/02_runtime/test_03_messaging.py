import asyncio

from cake.meta.test_utils.random_string import random_string
from cake import Robot
from std_msgs.msg import String
import rclpy.qos


def test_ros_messaging():

    robot = Robot({})
    assert robot.health()

    @robot.runtime.run_in_event_loop
    async def body():
        payload = f'test_message_{random_string()}'
        topic = '/test_topic_whrwbd' # Hard-coded for testing purposes: `ros2 topic echo /test_topic_whrwbd std_msgs/msg/String`

        publisher = robot.runtime.ros_interface.create_publisher(String, topic, rclpy.qos.HistoryPolicy.KEEP_LAST)
        def callback(msg):
            global incoming_msg
            incoming_msg = msg
        robot.runtime.ros_interface.create_subscription(String, topic, callback, rclpy.qos.HistoryPolicy.KEEP_LAST)

        outgoing_msg = String()
        outgoing_msg.data = payload
        publisher.publish(outgoing_msg)

        await asyncio.sleep(1e-3) # Note there's no spin call. It's handled in the runtime event loop.

        assert incoming_msg.data == payload

    for _ in range(10):
        body()  # type: ignore

    robot.shutdown()
