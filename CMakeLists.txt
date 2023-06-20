cmake_minimum_required(VERSION 3.0.2)
project(trochoids)

## Compile as C++17
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS)

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
    # By default, use Release mode
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type of build" FORCE)
endif()

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME}
  src/trochoids.cpp
  src/trochoid_utils.cpp
  src/DubinsStateSpace.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  add_executable(${PROJECT_NAME}-test test/unit_test_trochoids.cpp
                                        src/trochoids.cpp
                                        src/trochoid_utils.cpp
  )
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES} ${OMPL_LIBRARIES} GTest::gtest_main)
  endif()

  find_package(benchmark REQUIRED)
  add_executable(${PROJECT_NAME}-benchmark test/all_benchmarks.cpp)
  target_link_libraries(${PROJECT_NAME}-benchmark ${PROJECT_NAME} ${catkin_LIBRARIES} ${OMPL_LIBRARIES} benchmark::benchmark)
endif(CATKIN_ENABLE_TESTING)

