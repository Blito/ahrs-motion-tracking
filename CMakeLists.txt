project(myAHRS_accel)

cmake_minimum_required(VERSION 3.2)

# Build using C++14
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

add_definitions(-D_WIN32)
add_definitions(-D_USE_MATH_DEFINES)

set(myAHRS_SRC 
        "myahrs_plus.hpp"
        "Probe.h"
        "Probe.cpp"
        "main.cpp")

add_executable(myAHRS_accel ${myAHRS_SRC})

# GLM includes
include_directories("${CMAKE_SOURCE_DIR}/lib/glm/")
