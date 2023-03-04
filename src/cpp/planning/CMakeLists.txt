cmake_minimum_required(VERSION 3.16.3)
project(planning)
set(CMAKE_CXX_STANDARD 20)

include_directories(include)
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_library(planning
  src/path.cpp 
  src/trajectory.cpp
  src/robot_state.cpp 
  src/joint_state.cpp 
  src/pose_state.cpp 
  src/robot_state_collection.cpp
  src/time_scaler.cpp
  src/naive_robot_state_collection.cpp
  src/planning_request.cpp
  src/utilities.cpp
  src/rrt_base.cpp
  src/rrt.cpp
)

target_include_directories(planning PUBLIC ./include ../robot/include/)
target_link_libraries(planning PUBLIC robot geometry)

set(BUILD_TESTS True)
if (BUILD_TESTS)
  add_subdirectory(tests)
endif()
