cmake_minimum_required(VERSION 3.8)
project(turt_localization)
set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(<dependency> REQUIRED)

add_executable(turt_localize src/turt_localize.cpp)
ament_target_dependencies(turt_localize
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  rclcpp
  nav_msgs
  )
# target_include_directories(turt_localize PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)


install(TARGETS turt_localize
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package()

# put these above the auto_package if I end up putting them back...
# target_compile_features(turt_localize PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17



# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()
