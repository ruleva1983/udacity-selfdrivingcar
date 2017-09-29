
include_directories(third_party/googletest-release-1.8.0/googletest/include)
include_directories(third_party/googletest-release-1.8.0/googlemock/include)

add_executable(map_tests map_tests.cpp)
target_link_libraries(map_tests gtest gmock gmock_main)
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_link_libraries(map_tests python2.7)
endif()
