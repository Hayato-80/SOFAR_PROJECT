import launch
from launch import LaunchDescription
from launch_ros.actions import Node  # Correct import for Galactic
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

PACKAGE_NAME = 'navv'

def generate_launch_description():
    return LaunchDescription([
        

        Node(
            package='rviz2',
            executable='rviz2', 
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', PathJoinSubstitution([get_package_share_directory(PACKAGE_NAME), 'config', 'config.rviz'])]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['-5.5', '-5.5', '0', '0', '0', '0', 'map', 'odom']
        ),

        Node(
            package='turt_localization',
            executable='turt_localize',
            name='turt_localize',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='detect_obstacles',
            executable='create_map',
            name='create_map_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package=PACKAGE_NAME,
            executable='driving_node',
            name='driving_node',
            output='screen',
            parameters=[PathJoinSubstitution([get_package_share_directory(PACKAGE_NAME), 'config', 'driving_node.yaml']),
                {'use_sim_time': True}
            ]
        )


    ])
