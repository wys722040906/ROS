cmake_minimum_required(VERSION 3.0.2)
project(demo03_topic)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  demo07_auto_msg
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo_pkg
 CATKIN_DEPENDS roscpp rospy std_msgs demo07_auto_msg
#  DEPENDS system_lib
)

add_executable(topic_puber src/topic_puber.cpp)
add_executable(topic_suber src/topic_suber.cpp)

target_link_libraries(topic_puber
  ${catkin_LIBRARIES}
)
target_link_libraries(topic_suber
  ${catkin_LIBRARIES}
)
# target_link_libraries(topic_pub_p
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(topic_sub_p
#   ${catkin_LIBRARIES}
# )


catkin_install_python(PROGRAMS
  scripts/topic_pub.py 
  scripts/topic_sub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
