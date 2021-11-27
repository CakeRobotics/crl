from threading import Thread
from time import sleep
from launch import LaunchService

def launcher_process_main(launch_description_queue):
    launch_service = LaunchService()
    def queue_listener():
        while True:
            launch_description = launch_description_queue.get()
            launch_service.include_launch_description(launch_description)
    listener_thread = Thread(target=queue_listener)
    listener_thread.start()
    # sleep(4)  # FIXME (it was needed before but somehow got fixed.)
    launch_service.run(shutdown_when_idle=False)
