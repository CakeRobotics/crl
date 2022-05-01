from os.path import join

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml

from cake.utils.deep_update import deep_update

def generate_launch_description(props):
    use_sim_time = props.get('sim') or False
    robot_radius = props.get('body_size') or 0.5
    bringup_dir = get_package_share_directory('nav2_bringup')
    with open(join(bringup_dir, 'params', 'nav2_params.yaml')) as f:
        params = yaml.safe_load(f)

    params = deep_update(params, {
        'controller_server': {
            'ros__parameters': {
                'general_goal_checker': {
                    'xy_goal_tolerance': 0.25, # Note the floating point... TODO: Make dynamic based on robot size
                    'yaw_goal_tolerance': 1.67 # TODO: Reduce / Fix / Make dynamic
                }
            }
        },
        'local_costmap': {
            'local_costmap': {
                'ros__parameters': {
                    'robot_radius': robot_radius
                }
            }
        },
        'global_costmap': {
            'global_costmap': {
                'ros__parameters': {
                    'robot_radius': robot_radius
                }
            }
        },
    })
    params_path = '/tmp/cake_navigation_params.yaml'
    with open(params_path, 'w') as f:
        yaml.safe_dump(params, f)

    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(
            get_package_share_directory('nav2_bringup'),
            'launch',
            'navigation_launch.py'
        )),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'params_file': params_path,
        }.items(),
    )
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(
            get_package_share_directory('slam_toolbox'),
            'launch',
            'online_async_launch.py'
        )),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
        }.items(),
    )
    return launch.LaunchDescription([
        navigation_node,
        slam_node,
    ])
