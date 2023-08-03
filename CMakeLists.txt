cmake_minimum_required(VERSION 3.0)

project(puara_controller)

find_package(PkgConfig REQUIRED)
find_package(SDL2 REQUIRED)
find_library(LIBLO liblo lo REQUIRED)
find_library(JSONCPP jsoncpp REQUIRED)

include_directories(${SDL2_INCLUDE_DIRS} ${JSON_INC_PATH} ${JSONCPP_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR})

add_library(puara_controller.hpp puara_controller.cpp)

add_executable(${PROJECT_NAME} puara_controller.cpp main.cpp)
target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARIES} ${JSONCPP_LIBRARIES} -ljsoncpp liblo.a)