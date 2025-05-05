from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = 'nnavigation'

    map_file = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'maps',
        'map.yaml'
    ])

    params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nav2_params2.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nnavigation.rviz'
    )

    bringup_launch = PathJoinSubstitution([
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    static1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'waffle2/odom']
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
    #             'use_sim_time': False,
    #             'frame_id': 'map'
    #         }]
    #     )]
    # )



    return LaunchDescription([
        rviz_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'map': map_file,
                'params_file': params_path,
                'use_sim_time': 'false'
            }.items()
        ),

        
        # robot_state_publisher,
        static1
                ])
