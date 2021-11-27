from launch import LaunchDescription
from launch.actions import ExecuteProcess
from os.path import dirname, realpath

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'python3',
                'echo_chamber.py'
            ],
            output='screen',
            cwd=dirname(realpath(__file__)),
        ),
    ])
