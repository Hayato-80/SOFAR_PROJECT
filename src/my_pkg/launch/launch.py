from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
import os
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = 'robonav'

    map_file = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'maps',
        'beacon_map.yaml'
    ])

    params_path = os.path.join(
        get_package_share_directory('my_nav2_package'),
        'config',
        'nav2_params.yaml'
    )

    # map_server_node = TimerAction(
    #     period=2.0,
    #     actions=[Node(
    #         package='nav2_map_server',
    #         executable='map_server',
    #         name='map_server',
    #         output='screen',
    #         parameters=[{
    #             'yaml_filename': map_file,
    #             'use_sim_time': True,
    #             'always_send_full_map': True,
    #             'frame_id': 'map'
    #         }]
    #     )]
    # )

    nav2_node = Node(
        package='nav2_bringup',
        executable='nav2_navigation_launch.py',
        name='nav2',
        output='screen',
        parameters=[params_path],
        arguments=['--ros-args', '--params-file', params_path, '--map', map_file],
    )

    return LaunchDescription([
        map_server_node,
        nav2_node,
    ])

