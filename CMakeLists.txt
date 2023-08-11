cmake_minimum_required(VERSION 3.0)

set( SDL_STATIC ON CACHE BOOL "" FORCE )
set( SDL_SHARED OFF CACHE BOOL "" FORCE )
option(WITH_STATIC "Build LIBLO static library." ON)

project(puara_controller)

add_subdirectory(include/jsoncpp)
add_subdirectory(include/liblo/cmake)
add_subdirectory(include/SDL)

add_executable(${PROJECT_NAME} puara_controller.hpp puara_controller.cpp main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/SDL/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/jsoncpp/include 
    ${CMAKE_CURRENT_BINARY_DIR}/include/liblo/cmake 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/liblo 
)

target_link_libraries(${PROJECT_NAME} SDL2-static jsoncpp_static liblo_static)