import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'navv'
RVIZ_CONFIG_PATH = PathJoinSubstitution([get_package_share_directory(PACKAGE_NAME), 'ros', 'config', 'kame.rviz'])

def generate_launch_description():

    # Nodes
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', PathJoinSubstitution([get_package_share_directory('detect_obstacles'), 'config', 'kame.rviz'])]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom_frame_fhj']
    )

    localization_node = Node(
        package='turt_localization',
        executable='turt_localize',
        name='turt_localize',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # obstacle_detection_node = Node(
    #     package='detect_obstacles',
    #     executable='create_map',
    #     name='create_map_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}],
    #     remappings=[
    #         ('/odom', '/odom_fhj')
    #     ]
    # )

    obstacle_detection_node = Node(
        package='detect_obstacles',
        executable='find_obstacles',
        name='find_obstacles',
        output='screen',
        parameters=[
            # PathJoinSubstitution([get_package_share_directory(PACKAGE_NAME), 'config', 'driving_node.yaml']),
            {'use_sim_time': True}
        ],
        remappings=[
            ('/odom', '/odom_fhj')
        ]
    )
    driving_node = Node(
        package='navv',
        executable='driving_node',
        name='driving_node',
        output='screen',
        parameters=[
            # PathJoinSubstitution([get_package_share_directory(PACKAGE_NAME), 'config', 'driving_node.yaml']),
            {'use_sim_time': True}
        ],
        remappings=[
            ('/odom', '/odom_fhj')
        ]
    )
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('/odom', '/odom_fhj')]
    )

    # Launch description
    ld = LaunchDescription()

    # Add actions to the launch description
    ld.add_action(rviz_node)
    ld.add_action(amcl_node)
    ld.add_action(static_tf_node)
    ld.add_action(localization_node)
    ld.add_action(obstacle_detection_node)
    ld.add_action(driving_node)

    return ld
