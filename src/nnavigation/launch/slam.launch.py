from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
import os




def generate_launch_description():

    config_path = os.path.join(
            get_package_share_directory('nnavigation'), 'config', 'slam2.rviz'
        )
    
    return LaunchDescription([

        

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0', '0', '0.1', '0', '0', '0', 'waffle2/base_link', 'waffle2/base_scan'],
        #     name='static_lidar_tf'
        # ),


        # Start SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'waffle2/odom',
                'base_frame': 'waffle2/base_link',
                'scan_topic': '/waffle2/scan',
                'map_frame': 'map',
                'range_min': 0.12,
                'range_max': 3.5
            }],

        ),

        

        # Start RViz (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', config_path],
        ),


    ])
