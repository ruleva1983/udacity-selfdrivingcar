project(PARTICLE_FILTER)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/particle_filter.cpp main.cpp)

include_directories(include)

add_executable(particle_filter ${sources})

target_link_libraries(particle_filter z ssl uv uWS)

