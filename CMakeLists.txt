cmake_minimum_required(VERSION 3.0)

project(puara_controller)

# Set SDL flags to build the static library
set( SDL_STATIC ON CACHE BOOL "" FORCE )
set( SDL_SHARED OFF CACHE BOOL "" FORCE )
#option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)

# option(JSONCPP_LIB_BUILD_STATIC "Build static libraries" ON)
# option(JSONCPP_LIB_BUILD_SHARED "Build shared libraries" OFF)

# Add git submodules for libraries
add_subdirectory(jsoncpp)
#add_subdirectory(include/liblo/cmake)
add_subdirectory(SDL)

# Add source files
#add_library( puara_controller.hpp puara_controller.cpp )

add_executable(${PROJECT_NAME} puara_controller.hpp puara_controller.cpp main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/SDL/include)

# Link libraries
#target_link_libraries(${PROJECT_NAME} STATIC SDL3-static SDL3::SDL3-static libSDL3.a jsoncpp_lib_static)
target_link_libraries(${PROJECT_NAME} STATIC SDL2-static SDL2::SDL2-static libSDL2.a jsoncpp_lib_static)