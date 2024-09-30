cmake_minimum_required(VERSION 3.0.2)
project(demo06_tf2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  geometry_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo06_tf2
#  CATKIN_DEPENDS roscpp rospy std_msgs tf2 tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(tf_pub_static src/tf_pub_static.cpp)
add_executable(tf_sub_static src/tf_sub_static.cpp)

add_executable(tf_pub_dynamic src/tf_pub_dynamic.cpp)
add_executable(tf_sub_dynamic src/tf_sub_dynamic.cpp)

add_executable(tf_sub_more src/tf_sub_more.cpp)

add_executable(tf_turtle_generate src/tf_turtle_generate.cpp)
add_executable(tf_pub_follow src/tf_pub_follow.cpp)
add_executable(tf_sub_follow src/tf_sub_follow.cpp)

target_link_libraries(tf_pub_static
  ${catkin_LIBRARIES}
)
target_link_libraries(tf_sub_static
  ${catkin_LIBRARIES}
)
target_link_libraries(tf_pub_dynamic
  ${catkin_LIBRARIES}
)
target_link_libraries(tf_sub_dynamic
  ${catkin_LIBRARIES}
)
target_link_libraries(tf_sub_more
  ${catkin_LIBRARIES}
)
target_link_libraries(tf_turtle_generate
  ${catkin_LIBRARIES}
)
target_link_libraries(tf_pub_follow
  ${catkin_LIBRARIES}
)
target_link_libraries(tf_sub_follow
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/tf_pub_static.py
  scripts/tf_sub_static.py
  scripts/tf_pub_dynamic.py
  scripts/tf_sub_dynamic.py
  scripts/tf_sub_more.py     
  scripts/tf_turtle_generate.py
  scripts/tf_pub_follow.py
  scripts/tf_sub_follow.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

