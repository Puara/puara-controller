cmake_minimum_required(VERSION 3.0)

project(Puara_joystick)

find_package(SDL2 REQUIRED)

include_directories(${SDL2_INCLUDE_DIRS} "/usr/include")

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARIES})
