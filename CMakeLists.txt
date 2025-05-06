cmake_minimum_required(VERSION 3.8)
project(arm_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro)

# Specify the ROS 2 version and setup dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)

# Add the source file for the plugin
add_library(my_gazebo_plugin SHARED
  src/joint_control.cpp
)

# Link the library with Gazebo and ROS 2 dependencies
target_link_libraries(my_gazebo_plugin
  ${GAZEBO_LIBRARIES}
  rclcpp::rclcpp
)

# Install the plugin
install(TARGETS my_gazebo_plugin
  DESTINATION lib/gazebo-plugins
)

install(DIRECTORY launch models urdf worlds
	DESTINATION share/${PROJECT_NAME})

ament_package()
