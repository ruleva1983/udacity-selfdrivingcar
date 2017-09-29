
include_directories(third_party/googletest-release-1.8.0/googletest/include)
include_directories(third_party/googletest-release-1.8.0/googlemock/include)


add_executable(sensor_fusion_tests
               sensor_fusion_tests.cpp)
               
target_link_libraries(sensor_fusion_tests gtest gmock gmock_main)

#if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
#    target_link_libraries(sensor_fusion_tests python2.7)
#endif()
