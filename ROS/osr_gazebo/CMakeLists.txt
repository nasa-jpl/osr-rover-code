cmake_minimum_required(VERSION 3.5.0)
project(osr_gazebo)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

install(DIRECTORY
  launch
  config
  urdf
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(osr_controller src/osr_controller.cpp)
ament_target_dependencies(osr_controller
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  tf2_geometry_msgs
  trajectory_msgs
)

install(
  TARGETS
    osr_controller
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
