import asyncio

from cake.meta.test_utils.random_string import random_string
from cake import Robot
from std_msgs.msg import String
import rclpy.qos

from external_node.echo_chamber_launch import generate_launch_description


def test_external_nodes():

    robot = Robot({})
    assert robot.health()

    @robot.runtime.run_in_event_loop
    async def body():
        launch_description = generate_launch_description()
        robot.runtime.ros_interface.launch_external_nodes(launch_description)
        await asyncio.sleep(10)

        payload = f'test_message_{random_string()}'

        publisher = robot.runtime.ros_interface.create_publisher(String, '/ec_in', rclpy.qos.HistoryPolicy.KEEP_LAST)
        def callback(msg):
            global incoming_msg
            incoming_msg = msg
        robot.runtime.ros_interface.create_subscription(String, '/ec_out', callback, rclpy.qos.HistoryPolicy.KEEP_LAST)

        outgoing_msg = String()
        outgoing_msg.data = payload
        publisher.publish(outgoing_msg)

        await asyncio.sleep(1)

        assert incoming_msg.data == payload

    body()  # type: ignore

    robot.shutdown()
