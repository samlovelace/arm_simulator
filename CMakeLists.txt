cmake_minimum_required(VERSION 3.1)
project(robot_simulator)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

include(plugins/jointPositionController/CMakeLists.txt)
include(plugins/posePublisher/CMakeLists.txt)
include(plugins/vehicleController/CMakeLists.txt)

include(sim/CMakeLists.txt)

# install common files 
install(DIRECTORY launch models urdf worlds DESTINATION share/${PROJECT_NAME})

ament_package()
