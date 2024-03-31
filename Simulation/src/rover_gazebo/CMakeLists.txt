cmake_minimum_required(VERSION 3.0.2)
project(rover)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf 
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  tf
  nav_msgs
  tf2_ros
  )

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   xbox.msg
#   Message2.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rover
#  CATKIN_DEPENDS urdf
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_controller src/motor_controller.cpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES})

add_executable(rover_xbox src/rover_xbox.cpp)
target_link_libraries(rover_xbox ${catkin_LIBRARIES})

add_executable(xbox_joy src/xbox_joy.cpp)
target_link_libraries(xbox_joy ${catkin_LIBRARIES})
