cmake_minimum_required(VERSION 3.16)
project(librobot_traj_generate)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(Eigen3 3.3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS other_catkin_pkg
   DEPENDS Eigen3
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)


## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/librobot_traj_generate.cpp
 )


## Unit testing via G-Test
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(mytest test/src/gtest.cpp)
  target_link_libraries(mytest ${PROJECT_NAME} ${catkin_LIBRARIES} pthread)
endif(CATKIN_ENABLE_TESTING)
