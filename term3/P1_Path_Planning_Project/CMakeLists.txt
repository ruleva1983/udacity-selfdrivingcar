cmake_minimum_required (VERSION 3.5)
project(path_planning)


add_definitions(-std=c++11 -g -lpython2.7)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

option(build_tests "Building tests..." ON)


set(sources src/main.cpp)
            #src/map.cpp
            #src/sensor_fusion.cpp
            #src/agent.cpp
            #src/behavior.cpp
            #src/trajectory.cpp)


include_directories(/usr/local/include)
include_directories(include
                    third_party/spline
                    third_party/Eigen)
                    #third_party/matplotlib)
link_directories(/usr/local/lib)   

add_executable(path_planning ${sources})


target_link_libraries(path_planning z ssl uv uWS)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_link_libraries(path_planning python2.7)
endif()


if(build_tests)
    add_subdirectory(third_party/googletest-release-1.8.0)
    add_subdirectory(tests)
endif()
