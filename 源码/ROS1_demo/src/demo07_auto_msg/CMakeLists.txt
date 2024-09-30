cmake_minimum_required(VERSION 3.0.2)
project(demo07_auto_msg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
)
#引用Boost  C++库
#find_package(Boost REQUIRED COMPONENTS system) 

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  auto_msg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Specify additional locations of header files
## Your package locati_ons should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo_pkg
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
#  DEPENDS system_lib
)

# add_executable(${PROJECT_NAME}_node src/demo07_auto_msg_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
