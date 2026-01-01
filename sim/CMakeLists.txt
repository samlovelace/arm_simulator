cmake_minimum_required(VERSION 3.8)
#project(robot_simulator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_idl REQUIRED)
find_package(Eigen3 REQUIRED)

set(SOURCES 
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/UdpServer.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
#    ${robot_idl_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    robot_idl
)

install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)