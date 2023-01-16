include_directories(include)
#find_package(geometry)


add_library(robot
            src/robot.cpp 
            src/UR5.cpp
            src/joint.cpp 
            src/link.cpp
            #src/settings.cpp
            )


include_directories(../geometry/include)
target_link_libraries(robot PUBLIC geometry)

set(BUILD_TESTS True)
if (BUILD_TESTS)
  add_subdirectory(tests)
endif()

