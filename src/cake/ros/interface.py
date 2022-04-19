import asyncio
import logging
from multiprocessing import Process, Queue
import signal
from time import sleep
import traceback

from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.parameter import Parameter
import tf2_ros

from .launcher_process_main import launcher_process_main
from cake.utils.is_running_as_test import is_running_as_test

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
        self.__node__.destroy_node()

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

    # TODO: make async?
    # TODO: log if stuck
    def wait_for_node_to_activate(self, node_name):
        client = self.__node__.create_client(GetState, f'{node_name}/get_state')
        while not client.wait_for_service(timeout_sec=1.0):
            pass
        req = GetState.Request()
        while True:
            future = client.call_async(req)
            while not future.done():
                rclpy.spin_once(self.__node__, timeout_sec=0)
            state = future.result().current_state.label
            if state == 'active':
                break
            elif state == 'inactive' and is_running_as_test():
                logging.info("Allowed inactive node only for the test environment.")
                break

    def stop_launcher_process(self):
        # Stabilize
        sleep(0.5)
        # SEND SIGINT
        self._launcher_process._check_closed()
        self._launcher_process._popen._send_signal(signal.SIGINT)

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

    def init_tf(self, use_sim_time):
        self.__node__.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time or False)])
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.__node__)

    def lookup_transform(self, target_frame, source_frame, *args, **kwargs):
        time = rclpy.time.Time()
        return self.tf_buffer.lookup_transform(target_frame, source_frame, time, *args, **kwargs)
