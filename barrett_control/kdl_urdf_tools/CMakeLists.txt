cmake_minimum_required(VERSION 2.8.3)
project(kdl_urdf_tools)

find_package(orocos_kdl REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
  roscpp kdl_parser std_msgs sensor_msgs geometry_msgs trajectory_msgs tf tf_conversions angles)

catkin_package(
  DEPENDS kdl
  CATKIN_DEPENDS roscpp kdl_parser std_msgs sensor_msgs geometry_msgs trajectory_msgs tf tf_conversions angles
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})#
add_library(${PROJECT_NAME} src/tools.cpp)

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )

