cmake_minimum_required(VERSION 2.8.3)
project(geodetic_utils)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(geographiclib_conversions REQUIRED COMPONENTS)

cs_add_executable(geodetic_to_local_conversion_node src/geodetic_to_local_conversion_node.cpp)
#target_link_libraries(gps_to_pose_conversion_node  ${catkin_LIBRARIES})

cs_install()
cs_install_scripts(src/gps_spoofer.py src/euler_radians_to_degrees.py)
cs_export()
