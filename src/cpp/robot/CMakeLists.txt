include_directories(include)

add_library(robot
            src/robot.cpp 
            src/UR5.cpp
            src/joint.cpp 
            src/link.cpp
            src/utilities.cpp
            )

#find_package(Geometry)
#include_directories(${geometry_INCLUDE_DIRS}
if(NOT DEFINED ${geometry_INCLUDE_DIRS})
  set(geometry_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../geometry/include/" CACHE INTERNAL "")
endif(NOT DEFINED ${geometry_INCLUDE_DIRS})

include_directories(${geometry_INCLUDE_DIRS})
target_link_libraries(robot PUBLIC geometry)

set(BUILD_TESTS True)
if (BUILD_TESTS)
  add_subdirectory(tests)
endif()