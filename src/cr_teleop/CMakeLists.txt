cmake_minimum_required(VERSION 3.0.2)
project(cr_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)
#add_executable(cr_robot1_teleop src/cr_robot1_teleop.launch)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cr_teleop
#  CATKIN_DEPENDS ros-melodic-teleop-twist-keyboard roscpp rospy
#  DEPENDS system_lib
)
# include
#  ${catkin_INCLUDE_DIRS}
#)

