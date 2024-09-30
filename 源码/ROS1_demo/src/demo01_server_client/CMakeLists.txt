cmake_minimum_required(VERSION 3.0.2)
project(demo01_server_client)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  AddInt.srv
)

generate_messages(
  DEPENDENCIES
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



add_executable(AddInt_Server src/AddInt_Server.cpp)
add_executable(AddInt_Client src/AddInt_Client.cpp)
# add_executable(AddInt_Server_p src/AddInt_Server.cpp)
# add_executable(AddInt_Client_p src/AddInt_Client.cpp)

add_dependencies(AddInt_Server ${PROJECT_NAME}_gencpp)
add_dependencies(AddInt_Client ${PROJECT_NAME}_gencpp)
# add_dependencies(AddInt_Server_p ${PROJECT_NAME}_gencpp)
# add_dependencies(AddInt_Client_p ${PROJECT_NAME}_gencpp)

target_link_libraries(AddInt_Server
  ${catkin_LIBRARIES}
)
target_link_libraries(AddInt_Client
  ${catkin_LIBRARIES}
)

# target_link_libraries(AddInt_Server_p
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(AddInt_Client_p
#   ${catkin_LIBRARIES}
# )




catkin_install_python(PROGRAMS
  scripts/AddInt_Server_p.py 
  scripts/AddInt_Client_p.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
