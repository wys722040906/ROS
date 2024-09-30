cmake_minimum_required(VERSION 3.0.2)
project(demo04_turtlrsim)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo_pkg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_executable(turtle_control src/turtle_control.cpp)
add_executable(turtle_call src/turtle_call.cpp)
add_executable(turtle_srv src/turtle_srv.cpp)
add_executable(turtle_set_param src/turtle_set_param.cpp)

target_link_libraries(turtle_control
  ${catkin_LIBRARIES}
)
target_link_libraries(turtle_call
  ${catkin_LIBRARIES}
)
target_link_libraries(turtle_srv
  ${catkin_LIBRARIES}
)
target_link_libraries(turtle_set_param
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/turtle_control.py
  scripts/turtle_call.py  
  scripts/turtle_srv.py  
  scripts/turtle_set_param.py  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

