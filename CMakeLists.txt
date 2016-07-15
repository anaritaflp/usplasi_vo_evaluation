cmake_minimum_required(VERSION 2.8.3)
project(evaluate_vo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  geographic_msgs
  tf
  message_generation
  msgs_septentrio
  geodesy
)

add_message_files(
  FILES
  septentrio_asterx2.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs nav_msgs std_msgs msgs_septentrio geographic_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES evaluate_vo
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf geodesy
  DEPENDS system_lib
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#######################################################
# LIBRARY: Writing into matlab files

add_library(writeMatlab_LIB
  src/writeMatlabFile.cpp
  src/writeMatlabFile.h
)

target_link_libraries(writeMatlab_LIB
    ${catkin_LIBRARIES}
)

#######################################################
# EXECUTABLE: Writing VO and GT data into matlab file

add_executable(get_vogt_node 
    src/getMatlabDataVoGt.cpp
)

add_dependencies(get_vogt_node 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(get_vogt_node 
    ${catkin_LIBRARIES}
    writeMatlab_LIB
)

#######################################################
# EXECUTABLE: Writing only VO data into matlab file

add_executable(get_vo_node 
    src/getMatlabDataVo.cpp
)

add_dependencies(get_vo_node 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(get_vo_node 
    ${catkin_LIBRARIES}
    writeMatlab_LIB
)
