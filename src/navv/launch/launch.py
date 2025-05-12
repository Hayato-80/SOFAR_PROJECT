import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'navv'

def generate_launch_description():
    # Directories and configurations
    # launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    # pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    # x_pose = LaunchConfiguration('x_pose', default='0.0')
    # y_pose = LaunchConfiguration('y_pose', default='0.0')

    # world = os.path.join(
    #     get_package_share_directory('turtlebot3_gazebo'),
    #     'worlds',
    #     'empty_world.world'
    # )

    # # Gazebo server and client
    # gzserver_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    #     ),
    #     launch_arguments={'world': world}.items()
    # )

    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    #     )
    # )

    # # Robot state publisher
    # robot_state_publisher_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
    #     ),
    #     launch_arguments={'use_sim_time': use_sim_time}.items()
    # )

    # # Spawn TurtleBot
    # spawn_turtlebot_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
    #     ),
    #     launch_arguments={
    #         'x_pose': x_pose,
    #         'y_pose': y_pose
    #     }.items(),
    #     # remappings=[
    #     #     ('/odom', '/odom_fhj')
    #     # ]
    # )

    # Nodes
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-d', PathJoinSubstitution([get_package_share_directory(PACKAGE_NAME), 'config', 'config.rviz'])]
    )



    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom_frame_fhj']
    )

    static_tf_node_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['-0.064', '0', '0.132', '0', '0', '0', 'waffle2/base_footprint_fhj', 'waffle2/base_scan']
    )

    localization_node = Node(
        package='turt_localization',
        executable='turt_localize',
        name='turt_localize',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    obstacle_detection_node = Node(
        package='detect_obstacles',
        executable='find_obstacles',
        name='find_obstacles_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/waffle2/odom', '/waffle2/odom_fhj')
        ]
    )

    obstacle_detection_node = Node(
        package='detect_obstacles',
        executable='find_obstacles',
        name='find_obstacles',
        output='screen',
        parameters=[
            # PathJoinSubstitution([get_package_share_directory(PACKAGE_NAME), 'config', 'driving_node.yaml']),
            {'use_sim_time': True}
        ]
        # remappings=[
        #     ('/odom', '/odom_fhj')
        # ]
    )

    driving_node = Node(
        package=PACKAGE_NAME,
        executable='driving_node',
        name='driving_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([get_package_share_directory(PACKAGE_NAME), 'config', 'driving_node.yaml']),
            {'use_sim_time': False}
        ],
        # remappings=[
        #     ('/waffle2/odom', '/waffle2/odom_fhj')
        # ]
    )

    # Launch description
    ld = LaunchDescription()

    # Add actions to the launch description
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)
    # ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(rviz_node)
    ld.add_action(static_tf_node)
    ld.add_action(static_tf_node_2)
    ld.add_action(localization_node)
    ld.add_action(obstacle_detection_node)
    ld.add_action(driving_node)

    return ld
