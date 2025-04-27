cmake_minimum_required(VERSION 3.8)
project(ros2_service_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)

# Include directories
include_directories(include)

# Create executable for service server
add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server
  rclcpp
  example_interfaces
)

# Create executable for service client
add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client
  rclcpp
  example_interfaces
)

# Install executables
install(TARGETS
  service_server
  service_client
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
