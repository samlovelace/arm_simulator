cmake_minimum_required(VERSION 3.1)
project(arm_simulator)

add_subdirectory(plugins/jointPositionController)
add_subdirectory(plugins/posePublisher)

# install common files 
install(DIRECTORY launch models urdf worlds DESTINATION share/${PROJECT_NAME})
