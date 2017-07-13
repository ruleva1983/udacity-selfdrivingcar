project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources main.cpp 
    src/dynamics.cpp
    src/sigmapoints.cpp
    src/measurements.cpp
    src/filter.cpp
    src/tools.cpp)

include_directories(third-party)
include_directories(include)

add_executable(UnscentedKF ${sources})

target_link_libraries(UnscentedKF z ssl uv uWS)

if(build_tests)
    add_subdirectory(third-party/googletest-release-1.8.0)
    add_subdirectory(tests)
endif()
