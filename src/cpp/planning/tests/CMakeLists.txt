cmake_minimum_required(VERSION 3.16.3)

add_executable(temporary_test ./temporary_junk.cpp)
target_include_directories(temporary_test PUBLIC ${roscpp_INCLUDE_DIRS})
target_link_libraries(temporary_test planning collision_checking ${roscpp_LIBRARIES} visualizer geometry ros_conversions robot collision_checking)
