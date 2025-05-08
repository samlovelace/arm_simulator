cmake_minimum_required(VERSION 3.8)
project(arm_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(joint_command_plugin SHARED src/JointCommandSystemPlugin.cpp)
ament_target_dependencies(joint_command_plugin
  ignition-gazebo6
  rclcpp
  std_msgs
)

include_directories(joint_command_plugin 
  ${CMAKE_CURRENT_LIST_DIR}/include
)

install(TARGETS joint_command_plugin
        LIBRARY DESTINATION lib)

install(DIRECTORY launch meshes urdf worlds
DESTINATION share/${PROJECT_NAME})

ament_package()