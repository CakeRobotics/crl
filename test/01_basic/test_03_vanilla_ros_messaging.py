import rclpy
from std_msgs.msg import String

from cake.meta.test_utils.random_string import random_string

def test_async_messaging():
    payload = f'test_message_{random_string()}'
    topic = '/test_topic_hgppdr' # Hard coded for testing purposes: `ros2 topic echo /test_topic_hgppdr std_msgs/msg/String`

    rclpy.init()

    publisher_node = rclpy.create_node(f'test_publisher_node_{random_string()}')
    publisher = publisher_node.create_publisher(String, topic, 10)

    subscriber_node = rclpy.create_node(f'test_subscriber_node_{random_string()}')
    def callback(msg):
        global incoming_msg
        incoming_msg = msg
    subscriber_node.create_subscription(String, topic, callback, 10)

    outgoing_msg = String()
    outgoing_msg.data = payload
    publisher.publish(outgoing_msg)

    rclpy.spin_once(publisher_node, timeout_sec=0) # Not really necessary in this case
    rclpy.spin_once(subscriber_node, timeout_sec=0)

    assert incoming_msg.data == payload

    rclpy.shutdown()
