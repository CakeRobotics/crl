import launch
import launch_ros

def generate_launch_description(props):
    env_config = launch.actions.SetEnvironmentVariable(
        'ROS_MASTER_URI',
        f"http://127.0.0.1:{props['ros1_port']}",
    )
    bridge_node = launch_ros.actions.Node(
        package='ros1_bridge',
        executable='dynamic_bridge',
        arguments=['--bridge-all-2to1-topics'],
    )
    return launch.LaunchDescription([
        env_config,
        bridge_node,
    ])
