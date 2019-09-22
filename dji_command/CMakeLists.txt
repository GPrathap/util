cmake_minimum_required(VERSION 2.8.3)
project(dji_command)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  tf2
  tf2_geometry_msgs
  control_toolbox
  dynamic_reconfigure
  )
include_directories(include ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
  cfg/LidarAllign.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS roscpp
    DEPENDS
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
