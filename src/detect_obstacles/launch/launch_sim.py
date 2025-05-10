import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import  PathJoinSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch TurtleBot3 Gazebo world
    turtlebot3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_gazebo"),
                "launch",
                "turtlebot3_world.launch.py"
            ])
        ]),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Launch your custom node (delay to ensure /clock starts)
    potential_field_node = TimerAction(
        period=5.0,  # Delay to let /clock start from Gazebo
        actions=[
            Node(
                package="detect_obstacles",  # <-- Replace with your actual package
                executable="potential_field_node",  # <-- Replace with your executable
                name="potential_field_node",
                parameters=[{"use_sim_time": True}],
                output="screen"
            )
        ]
    )

    turt_localize_sim = TimerAction(
        period=5.0,  # Delay to let /clock start from Gazebo
        actions=[
            Node(
                package="turt_localization",  # <-- Replace with your actual package
                executable="turt_localize_sim",  # <-- Replace with your executable
                name="turt_localize_sim",
                parameters=[{"use_sim_time": True}],
                output="screen"
            )
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_gui')),
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        turtlebot3_world,
        potential_field_node,
        turt_localize_sim,
        joint_state_publisher,
    ])
