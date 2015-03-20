cmake_minimum_required(VERSION 2.8.3)
project(agv)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" -std=c++0x)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation tf)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   uint8Array.msg
   int16Array.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   kinematics.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mobile_robot
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(agv
#   src/${PROJECT_NAME}/agv.cpp
# )

## Declare a cpp executable
# add_executable(agv_node src/agv_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(agv_node agv_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(agv_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS agv agv_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_agv.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


add_executable(kinematics_server src/kinematics_server.cpp)
target_link_libraries(kinematics_server ${catkin_LIBRARIES})
add_dependencies(kinematics_server agv_gencpp)

add_executable(kinematics_client src/kinematics_client.cpp)
target_link_libraries(kinematics_client ${catkin_LIBRARIES})
add_dependencies(kinematics_client agv_gencpp)

add_executable(sharp_listener src/sharp_listener.cpp)
target_link_libraries(sharp_listener ${catkin_LIBRARIES})
add_dependencies(sharp_listener agv_gencpp)

add_executable(order_listener src/order_listener.cpp)
target_link_libraries(order_listener ${catkin_LIBRARIES})
add_dependencies(order_listener agv_gencpp)

add_executable(sharp_listener_publisher src/sharp_listener_publisher.cpp)
target_link_libraries(sharp_listener_publisher ${catkin_LIBRARIES})

add_executable(jsPublisher src/jsPublisher.cpp)
target_link_libraries(jsPublisher ${catkin_LIBRARIES})
add_dependencies(jsPublisher agv_gencpp)

add_executable(odomTF src/odomTF.cpp)
target_link_libraries(odomTF ${catkin_LIBRARIES})
add_dependencies(odomTF agv_gencpp)

add_executable(rectangular_tf src/rectangular_tf.cpp)
target_link_libraries(rectangular_tf ${catkin_LIBRARIES})
add_dependencies(rectangular_tf agv_gencpp)

add_executable(rectangular_tf_cl src/rectangular_tf_cl.cpp)
target_link_libraries(rectangular_tf_cl ${catkin_LIBRARIES})
add_dependencies(rectangular_tf_cl agv_gencpp)

add_executable(rectangular_tf_alt src/rectangular_tf_alt.cpp)
target_link_libraries(rectangular_tf_alt ${catkin_LIBRARIES})
add_dependencies(rectangular_tf_alt agv_gencpp)

add_executable(timer_deneme src/timer_deneme.cpp)
target_link_libraries(timer_deneme ${catkin_LIBRARIES})
add_dependencies(timer_deneme agv_gencpp)

add_executable(rectangular_trajectory_cont src/rectangular_trajectory_cont.cpp)
target_link_libraries(rectangular_trajectory_cont ${catkin_LIBRARIES})
add_dependencies(rectangular_trajectory_cont agv_gencpp)

add_executable(agvTeleop src/agvTeleop.cpp)
target_link_libraries(agvTeleop ${catkin_LIBRARIES})
add_dependencies(agvTeleop agv_gencpp)

add_executable(agvTeleopnew src/agvTeleopnew.cpp)
target_link_libraries(agvTeleopnew ${catkin_LIBRARIES})
add_dependencies(agvTeleopnew agv_gencpp)

add_executable(odomTF_better src/odomTF_better.cpp)
target_link_libraries(odomTF_better ${catkin_LIBRARIES})
add_dependencies(odomTF_better agv_gencpp)

add_executable(broadcaster_lidar src/broadcaster_lidar.cpp)
target_link_libraries(broadcaster_lidar ${catkin_LIBRARIES})
add_dependencies(broadcaster_lidar agv_gencpp)

add_executable(broadcaster_lidar_new src/broadcaster_lidar_new.cpp)
target_link_libraries(broadcaster_lidar_new ${catkin_LIBRARIES})
add_dependencies(broadcaster_lidar_new agv_gencpp)



