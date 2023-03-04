cmake_minimum_required(VERSION 3.16.3)

add_executable(temporary_test ./temporary_junk.cpp)
target_link_libraries(temporary_test planning collision_checking)
