import time

from cake import Robot
from cake.meta.test_utils.random_string import random_string
from std_msgs.msg import String


def test_ros_messaging():
    payload = f'test_message_{random_string()}'
    topic = '/test_topic_whrwbd' # Hard-coded for testing purposes: `ros2 topic echo /test_topic_whrwbd std_msgs/msg/String`

    robot = Robot()
    assert robot.health()

    publisher = robot.runtime.ros_interface.create_publisher(String, topic, 10, thread_check=False)
    def callback(msg):
        global incoming_msg
        incoming_msg = msg
    robot.runtime.ros_interface.create_subscription(String, topic, callback, 10, thread_check=False)

    outgoing_msg = String()
    outgoing_msg.data = payload
    publisher.publish(outgoing_msg)

    # Note there's no spin call. It's handled in the runtime event loop.
    # Note: this is unsafe practice. This actually exists to test threading, not messaging.
    time.sleep(0.1)

    assert incoming_msg.data == payload

    robot.shutdown()
