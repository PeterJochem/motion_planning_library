find_package(GTest REQUIRED)

set(test_sources 
    test_UR5.cpp
)

add_executable(test_robot_library ${test_sources})

target_link_libraries(
    test_robot_library
    ${GTEST_LIBRARIES} 
    ${GTEST_MAIN_LIBRARIES} 
    pthread
    geometry
    robot
)

include(GoogleTest)
gtest_discover_tests(test_robot_library)