
cmake_minimum_required(VERSION 3.0.2)
project(scorbot_control)

##Compile as C++11, supported in ROS Kinetic and newer
##add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")



################################################
## Declare ROS messages, services and actions ##
################################################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  cmake_modules
  hardware_interface
  controller_manager
  control_msgs
  urdf
  std_msgs
  ros_control_boilerplate
 #changes  
  actionlib
  controller_manager
  rosparam_shortcuts
  geometry_msgs
  
)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
    arm_command.msg
    scorbot_telemetry.msg
 )


## Generate services in th## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
  #std_msgs  
  #geometry_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   ccali_project_moveit_configfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )
#set(CMake_MODULE_PATH ${CMake_MODULE_PATH} $(PROJECT_SOURCE_DIR)/cmake)
#find_package(Gflags REQUIRED)

###################################
## catkin specific configuration ##
###################################


catkin_package(
 CATKIN_DEPENDS message_runtime roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scorbot_control
  CATKIN_DEPENDS std_msgs roscpp rospy
)
###########
## Build ##
###########
#include <scorbot_control/arm_command.msg>
#include <scorbot_control/scorbot_telemetry.msg>IRS}

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)



## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/gps_waypoint.cpp
# )

# add_executable(${PROJECT_NAME}_node src/gps_waypoint_node.cpp)

add_executable(scorbot_hw_main
  include/scorbot_control/scorbot_hw_interface.h
  src/scorbot_hw_main.cpp
  src/scorbot_hw_interface.cpp
  
)

add_library(scorbot_control 
  src/scorbot_hw_interface.cpp
)

add_dependencies(scorbot_hw_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(scorbot_hw_main
  ${catkin_LIBRARIES}
)



## same as for the library aboveadd_executable(scorbot_hw_main 

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
#include <scorbot_control/arm_command.msg>
#include <scorbot_control/scorbot_telemetry.msg>
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html




## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html


## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )







