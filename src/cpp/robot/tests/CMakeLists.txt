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

#include_directories(/usr/include/ginac/)
#target_link_libraries(test_robot_library /usr/lib/libginac.so)

#target_include_directories(geometry PUBLIC ../include/)
include(GoogleTest)
gtest_discover_tests(test_robot_library)
