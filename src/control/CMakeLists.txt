cmake_minimum_required(VERSION 3.0.2)
project(gnc_autonav)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES gnc_autonav
  CATKIN_DEPENDS 
    roscpp
    hardware_interface
    controller_manager
)

#add_executable(arm_hardware_interface src/arm_hardware_interface.cpp src/l298n.cpp)
#target_link_libraries(arm_hardware_interface ${catkin_LIBRARIES})
#link_directories(${catkin_LIBRARY_DIRS})

add_executable(test src/test.cpp)
target_link_libraries(test ${catkin_LIBRARIES})
link_directories(${catkin_LIBRARY_DIRS})
