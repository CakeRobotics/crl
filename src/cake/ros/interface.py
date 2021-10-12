import asyncio
import rclpy

class RosInterface:
    def __init__(self, runtime):
        self.runtime = runtime
        rclpy.init()
        self.__node__ = rclpy.create_node('cake_node')

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        rclpy.shutdown()

    async def spin_coro(self):
        while rclpy.ok():
            rclpy.spin_once(self.__node__, timeout_sec=0)
            await asyncio.sleep(1e-4)

    def create_publisher(self, *args, thread_check=True):
        if thread_check and not self.runtime.am_i_running_in_event_loop_thread():
            raise Exception('This function can only be called from event loop thread.')
        return self.__node__.create_publisher(*args)

    def create_subscription(self, *args, thread_check=True):
        if thread_check and not self.runtime.am_i_running_in_event_loop_thread():
            raise Exception('This function can only be called from event loop thread.')
        return self.__node__.create_subscription(*args)
