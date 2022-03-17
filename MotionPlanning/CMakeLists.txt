cmake_minimum_required (VERSION 2.8.11)
project (MotionPlanning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-w)

find_package(fcl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

include_directories(include)
#add_library(mazeSolving )

add_executable(main src/main.cpp)
target_link_libraries(main Eigen3::Eigen fcl)
#target_link_libraries(run_tests gtest gtest_main pthread mazeSolving)
