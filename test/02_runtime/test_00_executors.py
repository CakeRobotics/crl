import threading
from cake import Robot
import rclpy

def test_executors():
    robot = Robot({})
    assert robot.health()

    threads = threading.enumerate()

    backend_thread_list = list(filter(lambda t: t.name == 'backend_thread', threads))
    assert len(backend_thread_list) == 1
    backend_thread = backend_thread_list[0]
    assert backend_thread.is_alive()

    main_thread_list = list(filter(lambda t: t.name == 'MainThread', threads))
    assert len(main_thread_list) == 1
    main_thread = main_thread_list[0]
    assert main_thread.is_alive()

    launcher_process = robot.runtime.ros_interface._launcher_process
    assert launcher_process.is_alive()

    assert rclpy.ok()

    robot.shutdown()

    assert not backend_thread.is_alive()
    assert not launcher_process.is_alive()
    assert not rclpy.ok()
