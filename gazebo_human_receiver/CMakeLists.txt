cmake_minimum_required(VERSION 2.8.3)
project(gazebo_human_receiver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS geometry_msgs std_msgs roscpp rospy cv_bridge transitbuddy_msgs dynamic_reconfigure)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

MESSAGE( STATUS "GAZEBO_INCLUDE_DIRS:         " ${GAZEBO_INCLUDE_DIRS} )
MESSAGE( STATUS "SDFormat_INCLUDE_DIRS:       " ${SDFormat_INCLUDE_DIRS} )

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/human_receiver.cfg
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
   INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
   CATKIN_DEPENDS geometry_msgs std_msgs
   LIBRARIES lib
   v4r_gazebo_ros_factory
   v4r_gazebo_ros_del_obj
#  DEPENDS system_lib
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ./src
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
## Declare a cpp library
# add_library(v4r_gazebo
#   src/${PROJECT_NAME}/v4r_gazebo.cpp
# )

## Declare a cpp executable
#add_executable(joint_publisher src/joint_publisher.cpp)
add_library(gazebo_ros_human_receiver src/gazebo_ros_factory.cpp)

add_library(factory_and_fini src/factory_and_fini.cpp)


target_link_libraries(gazebo_ros_human_receiver  ${catkin_LIBRARIES})
target_link_libraries(factory_and_fini  ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(TARGETS joint_publisher
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
# install(TARGETS v4r_gazebo v4r_gazebo_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY 
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_v4r_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
