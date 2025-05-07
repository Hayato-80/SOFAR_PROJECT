from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import FindExecutable

def generate_launch_description():

    package_name = 'nnavigation'



    # with open(
    #     os.path.join(
    #         get_package_share_directory('turtlebot3_description'),
    #         'urdf',
    #         'turtlebot3_waffle2.urdf'
    #     ), 'r') as infp:
    #     urdf_content = infp.read()

    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': urdf_content}, 
    #                 {'use_sim_time': False}],
    #     remappings=[('/joint_states', '/waffle2/joint_states')]
    # )



    alias_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_alias',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'waffle2/base_link'],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # static_odom_to_basefootprint = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'waffle2/odom', 'waffle2/base_footprint'],
    #     output='screen',
    #     parameters=[{'use_sim_time': False}]
    # )



    map_file = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'maps',
        'map.yaml'
    ])

    params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nav2_params2_2.yaml'
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
        parameters=[{'use_sim_time': False}]
    )

    static1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'waffle2/odom'],
        parameters=[{'use_sim_time': False}]
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
        # robot_state_publisher_node,
        alias_base_link,
        # static_odom_to_basefootprint,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'map': map_file,
                'params_file': params_path,
                'use_sim_time': 'true'
            }.items()
        ),
        # robot_state_publisher,
        static1
                ])
