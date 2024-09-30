cmake_minimum_required(VERSION 3.0.2)
project(demo02_param_server)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo_pkg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(param_talker src/param_talker.cpp)
add_executable(param_listener src/param_listener.cpp)
add_executable(param_del src/param_del.cpp)
# add_executable(param_talker_p src/param_talker.cpp)
# add_executable(param_listener_p src/param_listener.cpp)
# add_executable(param_del_p src/param_del.cpp)



target_link_libraries(param_talker
  ${catkin_LIBRARIES}
)
target_link_libraries(param_listener
  ${catkin_LIBRARIES}
)
target_link_libraries(param_del
  ${catkin_LIBRARIES}
)
# target_link_libraries(param_talker_p
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(param_listener_p
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(param_del_p
#   ${catkin_LIBRARIES}
# )



catkin_install_python(PROGRAMS
  scripts/param_talker_p.py 
  scripts/param_listener_p.py
  scripts/param_del_p.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
