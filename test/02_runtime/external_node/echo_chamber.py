import rclpy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('echo_chamber')
    publisher = node.create_publisher(String, '/ec_out', rclpy.qos.HistoryPolicy.KEEP_LAST)
    def echo(msg):
        publisher.publish(msg)
        node.get_logger().info(f"Echoed message with data'{msg.data}'")
    node.create_subscription(String, '/ec_in', echo, rclpy.qos.HistoryPolicy.KEEP_LAST)
    node.get_logger().info('Echo chamber started.')
    rclpy.spin(node)

if __name__ == "__main__":
    main()
