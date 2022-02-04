from cake.utils.is_running_as_test import is_running_as_test
from cake.utils.get_gazebo_state import get_gazebo_state

def block_until_gazebo_runs(runtime):
    if is_running_as_test():
        return

    @runtime.run_in_event_loop
    def get_gazebo_state_body():
        node = runtime.ros_interface.__node__
        return get_gazebo_state(node)

    loop_first_time = True
    while True:
        state = get_gazebo_state_body()
        if state == 'RUNNING':
            break
        if loop_first_time:
            print('Simulation engine is paused/closed.')
            print('Start the simulation engine for robot code to execute.')
            loop_first_time = False
