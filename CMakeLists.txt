cmake_minimum_required(VERSION 3.1)
project(robot_simulator)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

#add_subdirectory(plugins/jointPositionController)
#add_subdirectory(plugins/posePublisher)
include(plugins/jointPositionController/CMakeLists.txt)
include(plugins/posePublisher/CMakeLists.txt)

# install common files 
install(DIRECTORY launch models urdf worlds DESTINATION share/${PROJECT_NAME})

ament_package()
