cmake_minimum_required(VERSION 2.8.3)
project(wpbh_local_planner)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    base_local_planner
    cmake_modules
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    roscpp
    std_msgs
    pluginlib
    tf
)

find_package(OpenCV REQUIRED)
 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wpbh_local_planner
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    std_msgs
    tf
    tf_conversions
  DEPENDS
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)

link_directories(./data)

install(FILES
  wpbh_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_library(wpbh_local_planner
  include/wpbh_local_planner/wpbh_local_planner.h
  include/wpbh_local_planner/CLidarAC.h
  include/wpbh_local_planner/wl_helper.h
  src/wpbh_local_planner.cpp
  src/CLidarAC.cpp
)
add_dependencies(wpbh_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpbh_local_planner
  ${catkin_LIBRARIES}  -lwl_helper
)