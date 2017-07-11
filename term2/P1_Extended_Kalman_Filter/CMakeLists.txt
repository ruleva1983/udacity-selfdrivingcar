project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources main.cpp 
    src/FusionEKF.cpp 
    src/extended_kalman_filter.cpp
    src/measurements.cpp 
    src/dynamics.cpp 
    src/tools.cpp)

include_directories(third-party)
include_directories(include)

add_executable(ExtendedKF ${sources})

target_link_libraries(ExtendedKF z ssl uv uWS)
