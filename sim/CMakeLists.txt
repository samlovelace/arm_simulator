cmake_minimum_required(VERSION 3.1)

find_package(Eigen3 REQUIRED)

set(SOURCES 
    ${CMAKE_CURRENT_LIST_DIR}/src/AbvUdpInputFetcher.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/UdpServer.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/include
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# install common files 
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})