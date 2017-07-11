project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources main.cpp 
    src/dynamics.cpp
    src/sigmapoints.cpp)

include_directories(third-party)
include_directories(include)

add_executable(UnscentedKF ${sources})

target_link_libraries(UnscentedKF z ssl uv uWS)
