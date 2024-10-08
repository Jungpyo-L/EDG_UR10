# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(edg_ur10)
find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

###################################
## catkin message generation     ##
###################################

#catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 SensorPacket.msg
 MarkerPacket.msg
 cmdPacket.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Enable.srv
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
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS rospy message_runtime roscpp
#    INCLUDE_DIRS include
)

#include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(c_model_ethercat_node 
#  src/robotiq_c_model_control/c_model_ethercat_node.cpp 
#  src/robotiq_c_model_control/c_model_ethercat_client.cpp
#)

#target_link_libraries(c_model_ethercat_node
#   ${robotiq_ethercat_LIBRARIES}
#   ${catkin_LIBRARIES}  
#)

#add_dependencies(c_model_ethercat_node robotiq_c_model_control_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

#install(PROGRAMS src/psocPubSub.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})