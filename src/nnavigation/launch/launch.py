from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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

    rviz_config_default = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
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
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    static1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'waffle2/odom']
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_default,
            description='Path to the RVIZ config file'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'map': map_file,
                'params_file': params_path,
                'use_sim_time': 'false'
            }.items()
        ),

        rviz_node,
        static1
                ])
