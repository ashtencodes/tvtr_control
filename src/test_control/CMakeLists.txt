cmake_minimum_required(VERSION 3.0.2)
project(test_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
    catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  controller_manager_msgs
  joint_state_controller
  std_msgs
  roscpp
  hardware_interface
  tf
)

find_library(PIGPIO pigpio)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    controller_manager
    roscpp
    hardware_interface
    controller_interface
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
    ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS scripts/pigpioInit.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(arm_hardware_interface src/arm_hardware_interface.cpp src/l298n.cpp)
target_link_libraries(arm_hardware_interface 
    ${catkin_LIBRARIES} 
    ${PIGPIO}
    ${CMAKE_DL_LIBS}
    ${LIBDISPATCH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${LD_LIBRARY_PATH})
link_directories(${catkin_LIBRARY_DIRS})