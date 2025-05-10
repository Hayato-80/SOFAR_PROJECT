cmake_minimum_required(VERSION 3.5)
project(navv)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake_auto REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED) # Add this line
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs  REQUIRED)

ament_auto_find_build_dependencies()

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}/
)

ament_auto_add_executable(driving_node src/driving_node.cpp)
ament_target_dependencies(driving_node rclcpp tf2_ros tf2_geometry_msgs nav_msgs geometry_msgs)


ament_auto_package()
