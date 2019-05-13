cmake_minimum_required(VERSION 2.8.3)
project(dji_command)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  dji_sdk
  roscpp
  tf
  tf2
  tf2_geometry_msgs
  control_toolbox
  dynamic_reconfigure
  mav_trajectory_generation
  mav_trajectory_generation_ros
  mav_planning_msgs
  mav_msgs
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

add_executable(set_trajectory_node
  src/set_trajectory_node.cpp
  )
target_compile_features(set_trajectory_node PRIVATE cxx_range_for)
target_link_libraries(set_trajectory_node ${catkin_LIBRARIES})
add_dependencies(set_trajectory_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)



install(TARGETS set_trajectory_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#add_executable(dji_command_node
#  src/dji_command_node.cpp
#  )
#target_compile_features(dji_command_node PRIVATE cxx_range_for)
#target_link_libraries(dji_command_node ${catkin_LIBRARIES})
#add_dependencies(dji_command_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


#install(TARGETS dji_command_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#add_executable(dji_identification_node
#  src/dji_identification_node.cpp
#  )
#target_compile_features(dji_identification_node PRIVATE cxx_range_for)
#target_link_libraries(dji_identification_node ${catkin_LIBRARIES})
#add_dependencies(dji_identification_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


#install(TARGETS dji_identification_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
