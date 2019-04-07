cmake_minimum_required(VERSION 2.8.3)
project(topic_viewer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  riki_msgs
  rviz
)
find_package(Qt5 COMPONENTS Core Widgets )
set(QT_LIBRARIES Qt5::Widgets)
#add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_plugin
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
set(CMAKE_AUTOMOC ON)
message(STATUS "Using Qt based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
qt5_wrap_cpp(
include/viewer/viewer.h
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})
set(SOURCE_FILES src/viewer.cpp include/viewer/viewer.h)

add_library(viewer ${SOURCE_FILES} )

target_link_libraries(viewer
   ${catkin_LIBRARIES}
   ${rviz_DEFAULT_PLUGIN_LIBRARIES}
   ${QT_LIBRARIES}
 )


install(TARGETS
  viewer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
