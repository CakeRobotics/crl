import asyncio
import functools
import threading
import time

from cake.ros.interface import RosInterface

class Runtime:
    def __init__(self):
        self.ros_interface_initialized = False
        self._shutting_down = False
        self.__loop__ = asyncio.new_event_loop()
        self.__loop_thread__ = threading.Thread(
            name='backend_thread',
            target=self.start_event_loop,
            daemon=True,
        )
        self.__loop_thread__.start()
        while not self.ros_interface_initialized:
            time.sleep(0.001)

    def start_event_loop(self):
        self.ros_interface = RosInterface(self)
        self.ros_interface.start_launcher_process()
        self.ros_interface_initialized = True
        self._ros_interface_task = self.__loop__.create_task(self.ros_interface.spin_internal_coro()) # This is only one of many tasks in the loop. This task controls main ROS node.
        self.__loop__.run_forever()
        if not self._shutting_down:
            raise self._ros_interface_task.exception()
            # os.kill(os.getpid(), signal.SIGINT)

    def assert_ros_interface_ok(self):
        task = self._ros_interface_task
        if task.done():
            raise task.exception()  # type: ignore

    def shutdown(self):
        # Enable _shutting_down flag so that backend_loop being stopped
        # won't be interprated as an error
        self._shutting_down = True

        # Shutdown rclpy which in turn makes rclpy.ok() = false, causing
        # the task in backend_loop to finish.
        self.__loop__.call_soon_threadsafe(self.ros_interface.shutdown)
        while not self._ros_interface_task.done():
            time.sleep(0.1)

        # Stop the backend loop which should be empty by now.
        # Checking __loop__.is_running is not really needed as it's already covered in the next line
        self.__loop__.call_soon_threadsafe(self.__loop__.stop)
        while self.__loop__.is_running():
            time.sleep(0.1)

        # backend_thread will automatically stop as soon as the loop finishes.
        self.__loop_thread__.join()

        # Stop external nodes process
        self.ros_interface.stop_launcher_process()
        self.ros_interface._launcher_process.join()

    def start_task(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.__loop__)

    def run_in_event_loop(self, async_function):
        @functools.wraps(async_function)
        def wrapper(*args, **kwargs):
            future = self.start_task(async_function(*args, **kwargs))
            while not future.done():
                time.sleep(0.01)
            # The next 3 lines aren't really necessary as result() already does what they do.
            exception = future.exception()
            if exception is not None:
                raise exception
            return future.result()
        return wrapper

    def am_i_running_in_event_loop_thread(self):
        return threading.current_thread().ident == self.__loop_thread__.ident

# This is like "runtime.run_in_event_loop", but is to be used to
# decorate member function of classes, as they don't have access
# to self.robot.runtime before initialization.
# The trick involves using self.robot.runtime, so make sure that
# your class assigns a reference to the Robot object at __init__.
def run_in_event_loop(async_function):
    @functools.wraps(async_function)
    def wrapper(self, *args, **kwargs):
        if self.robot.runtime.am_i_running_in_event_loop_thread():
            return async_function(self, *args, **kwargs) # Implicit! FIXME! This is to avoid deadlocks when running API functions within event loop threads. NOTE on FIXME: it still throws error if you don't await where you should or vice versa. So maybe it's not too bad!
        future = self.robot.runtime.start_task(async_function(self, *args, **kwargs))
        while not future.done():
            time.sleep(0.01)
        # The next 3 lines aren't really necessary as result() already does what they do.
        exception = future.exception()
        if exception is not None:
            raise exception
        return future.result()
    return wrapper
