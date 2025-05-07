from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your map and YAML file
    map_file = os.path.join(get_package_share_directory('nnavigation'), 'maps', 'map.yaml')

    # Path to RViz config file
    config_path = os.path.join(
        get_package_share_directory('nnavigation'), 'config', 'planning.rviz'
    )

    return LaunchDescription([

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_base_link',
            output='screen',
            arguments=['0', '0', '0.1', '0', '0', '0', 'map', 'waffle2/base_scan'],
            parameters=[{'use_sim_time': False}],
        ),

        # Start Map Server to publish the map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file, 'use_sim_time': False}],
        ),

        # Start RViz to visualize the map and robot
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', config_path],
            parameters=[{'use_sim_time': False}],
        ),

        # # Start the global planner (Navfn)
        # Node(
        #     package='nav2_planner',
        #     executable='planner_server',
        #     name='planner_server',
        #     output='screen',
        #     parameters=[{
        #         'expected_planner_frequency': 1.0,
        #         'planner_plugins': ["NavfnPlanner"],
        #         'NavfnPlanner': {
        #             'plugin': 'nav2_navfn_planner/NavfnPlanner'
        #         }
        #     }],
        # ),

        # # Start the local controller (Regulated Pure Pursuit)
        # Node(
        #     package='nav2_controller',
        #     executable='controller_server',
        #     name='controller_server',
        #     output='screen',
        #     parameters=[{
        #         'controller_frequency': 10.0,
        #         'controller_plugins': ["FollowPath"],
        #         'FollowPath': {
        #             'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
        #         }
        #     }],
        # ),

        # # Start RViz to visualize the map and robot
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', config_path],
        # ),

        # Lifecycle Manager to handle the lifecycle transitions
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            # parameters=[{'autostart': False, 'node_names': ['planner_server', 'controller_server', 'map_server']}],
            parameters=[{'autostart': False, 'node_names': ['map_server']}],
        ),
    ])
