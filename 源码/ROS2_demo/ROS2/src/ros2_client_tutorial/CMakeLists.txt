cmake_minimum_required(VERSION 3.8)
project(ros2_client_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2_interfaces_tutorial REQUIRED)

# include directories
include_directories(include)

# build nodes
add_executable(robot_status_publisher src/robot_status_publisher.cpp)
ament_target_dependencies(robot_status_publisher
  rclcpp
  ros2_interfaces_tutorial
)

add_executable(robot_move_server src/robot_move_server.cpp)
ament_target_dependencies(robot_move_server
  rclcpp
  ros2_interfaces_tutorial
)

add_executable(robot_move_client src/robot_move_client.cpp)
ament_target_dependencies(robot_move_client
  rclcpp
  ros2_interfaces_tutorial
)

add_executable(robot_navigator_server src/robot_navigator_server.cpp)
ament_target_dependencies(robot_navigator_server
  rclcpp
  rclcpp_action
  ros2_interfaces_tutorial
)

add_executable(robot_navigator_client src/robot_navigator_client.cpp)
ament_target_dependencies(robot_navigator_client
  rclcpp
  rclcpp_action
  ros2_interfaces_tutorial
)

# install executables
install(TARGETS
  robot_status_publisher
  robot_move_server
  robot_move_client
  robot_navigator_server
  robot_navigator_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
