cmake_minimum_required(VERSION 3.26.4)

# set(LIBLO_DIR "${BUILD_DIR}/liblo/liblo-master")
# set(LIBLO_BUILD_DIR "${LIBLO_DIR}/cmake/build/Release")
# set(LIBLO_INCLUDES "${LIBLO_DIR}/cmake/build;${LIBLO_DIR}")

project(Puara_joystick)

find_package(SDL2 REQUIRED)
find_library(LIBLO liblo lo REQUIRED)

include_directories("${SDL2_INCLUDE_DIRS}" "${CMAKE_CURRENT_LIST_DIR}")

add_library(puara_joystick.hpp puara_joystick.cpp)

add_executable(${PROJECT_NAME} puara_joystick.cpp main.cpp)
target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARIES} liblo.a)