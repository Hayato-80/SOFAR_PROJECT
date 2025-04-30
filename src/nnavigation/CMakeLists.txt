cmake_minimum_required(VERSION 3.5)
project(nnavigation)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake_auto REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rclpy REQUIRED) # Added this line
find_package(nav2_bringup REQUIRED) # Added this line
find_package(nav2_msgs REQUIRED) # Added this line
find_package(std_msgs REQUIRED) # Added this line
find_package(sensor_msgs REQUIRED) # Added this line

ament_auto_find_build_dependencies()

install(DIRECTORY launch config maps
    DESTINATION share/${PROJECT_NAME}/
)



ament_auto_package()
