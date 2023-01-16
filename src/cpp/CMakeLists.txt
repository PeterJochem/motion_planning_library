cmake_minimum_required(VERSION 3.16.3)
project(motion_planning_library)

#set(CMAKE_CXX_STANDARD 11)

add_subdirectory(robot)
add_subdirectory(geometry)
add_subdirectory(fcl_conversions)
