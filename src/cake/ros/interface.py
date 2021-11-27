import asyncio
from multiprocessing import Process, Queue
import traceback

import rclpy
from .launcher_process_main import launcher_process_main

class RosInterface:
    def __init__(self, runtime):
        self.runtime = runtime
        rclpy.init()
        self.__node__ = rclpy.create_node('cake_node')

    def __del__(self):
        pass
        # Note: the following code can clean up automatically:
        #
        # if rclpy.ok():
        #     self.shutdown()
        #
        # But it's currently disabled for the following reasons:
        #
        # 1. User may accidentaly let robot go out of scope.
        #    this will cause the robot to stop.
        #
        # 2. User may re-create another robot instance just
        #    after letting this robot out of context. Since del
        #    may run out of order, it may cause rclpy shutdown after
        #    the other robot has started rclpy, causing failure.
        #    Not using global objects or defining an object-specific
        #    flag may help with this.
        #
        # For now, shutdown must be explicitly called.

    def shutdown(self):
        rclpy.shutdown()

    async def spin_internal_coro(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self.__node__, timeout_sec=0)
                await asyncio.sleep(1e-4)
        except Exception:
            traceback.print_exc()
            self.runtime.__loop__.stop()
            raise

    def start_launcher_process(self):
        self._launch_description_queue = Queue()
        self._launcher_process = Process(
            target=launcher_process_main,
            args=[self._launch_description_queue],
            daemon=True,
        )
        self._launcher_process.start()

    def stop_launcher_process(self):
        self._launcher_process.kill()

    def create_publisher(self, *args, thread_check=True):
        if thread_check and not self.runtime.am_i_running_in_event_loop_thread():
            raise Exception('This function can only be called from event loop thread.')
        return self.__node__.create_publisher(*args)

    def create_subscription(self, *args, thread_check=True):
        if thread_check and not self.runtime.am_i_running_in_event_loop_thread():
            raise Exception('This function can only be called from event loop thread.')
        return self.__node__.create_subscription(*args)

    def launch_external_nodes(self, launch_description, thread_check=True):
        if thread_check and not self.runtime.am_i_running_in_event_loop_thread():
            raise Exception('This function can only be called from event loop thread.')
        self._launch_description_queue.put(launch_description)
