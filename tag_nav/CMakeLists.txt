cmake_minimum_required(VERSION 2.8.3)
project(tag_nav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  riki_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tag_nav
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tagNav src/tag_nav.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tagNav
   ${catkin_LIBRARIES}
)
