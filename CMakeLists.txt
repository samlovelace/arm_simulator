cmake_minimum_required(VERSION 3.8)
project(arm_simulator)
set(PLUGIN_NAME joint_position_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/src/JointPositionController.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/RateController.cpp
)

add_library(${PLUGIN_NAME} SHARED ${SOURCES})

ament_target_dependencies(${PLUGIN_NAME}
  ignition-gazebo6
  rclcpp
  std_msgs
)

include_directories(${PLUGIN_NAME} 
  ${CMAKE_CURRENT_LIST_DIR}/include
)

install(TARGETS ${PLUGIN_NAME}
        LIBRARY DESTINATION lib)

install(DIRECTORY launch models urdf worlds
DESTINATION share/${PROJECT_NAME})

ament_package()