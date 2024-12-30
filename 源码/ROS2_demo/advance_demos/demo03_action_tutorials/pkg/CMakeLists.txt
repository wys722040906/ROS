cmake_minimum_required(VERSION 3.5)
project(action_tutorials_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

# 设置动作文件目录
set(ACTION_FILES
  "action/Test.action"
)

#colcon build --symlink-install -- 不复制拷贝，创建符号链接编译


#默认生成路径
#<workspace>/install/<your_package_name>/include/<your_package_name>/action/
# 生成接口文件
rosidl_generate_interfaces(${PROJECT_NAME}
  ${ACTION_FILES}
  DEPENDENCIES std_msgs # 可根据需要指定依赖
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
