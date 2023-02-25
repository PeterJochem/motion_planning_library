cmake_minimum_required(VERSION 3.16.3)
project(CollisonChecking)
set(CMAKE_CXX_STANDARD 20)

include_directories(include)
find_package(GTest REQUIRED)
find_package(fcl REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

include_directories(/home/pj/ros_motion_planning/src/motion_planning_library/src/cpp/robot/include/robots)

add_library(collision_checking
  src/collision_checker.cpp
  src/robot_internal_collision_checker.cpp
  src/fcl_robot_internal_collision_checker.cpp
  src/conversions.cpp
)

target_include_directories(collision_checking PUBLIC ${FCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} /home/pj/ros_motion_planning/src/motion_planning_library/src/cpp/robot/include/robots)
target_link_libraries(collision_checking PUBLIC Eigen3::Eigen fcl robot)

set(BUILD_TESTS True)
if (BUILD_TESTS)
  add_subdirectory(tests)
endif()