cmake_minimum_required(VERSION 3.8)
project(arm_simulator)
set(ARM_PLUGIN_NAME joint_position_controller)
set(CAMERA_PLUGIN_NAME stereo_camera_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(ARM_SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/src/JointPositionController.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/RateController.cpp
)

set(CAMERA_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/src/RateController.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/StereoCameraPublisher.cpp
)

add_library(${ARM_PLUGIN_NAME} SHARED ${ARM_SOURCES})
add_library(${CAMERA_PLUGIN_NAME} SHARED ${CAMERA_SOURCES})

ament_target_dependencies(${ARM_PLUGIN_NAME}
  ignition-gazebo6
  rclcpp
  std_msgs
)

ament_target_dependencies(${CAMERA_PLUGIN_NAME}
  ignition-gazebo6
  rclcpp
)

include_directories(${ARM_PLUGIN_NAME} 
  ${CMAKE_CURRENT_LIST_DIR}/include
)

include_directories(${CAMERA_PLUGIN_NAME} 
  ${CMAKE_CURRENT_LIST_DIR}/include
)

install(TARGETS ${ARM_PLUGIN_NAME}
        LIBRARY DESTINATION lib)

install(TARGETS ${CAMERA_PLUGIN_NAME}
LIBRARY DESTINATION lib)


install(DIRECTORY launch models urdf worlds
DESTINATION share/${PROJECT_NAME})

ament_package()