from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            '/home/ferdinand/SOFAR_PROJECT/src/nnavigation/maps/map.yaml'),
        description='Full path to map yaml file'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'launch',
                'navigation2.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file
        }.items()
    )

    # Static TF: waffle2/odom -> odom
    # Static TF: waffle2/odom → odom
    tf_odom_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'waffle2/odom'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Static TF: waffle2/base_link → base_link
    tf_base_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'waffle2/base_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )


    return LaunchDescription([
        declare_use_sim_time,
        declare_map_file,
        tf_odom_to_odom,
        tf_base_to_base,
        nav2_launch
    ])
