import rclpy

def test_rclpy_init():
    rclpy.init()
    assert rclpy.ok()
    rclpy.shutdown()
