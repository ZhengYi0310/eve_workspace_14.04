
cmake_minimum_required(VERSION 2.8.3)
project(wam_node)
find_package(catkin REQUIRED COMPONENTS std_msgs std_srvs sensor_msgs geometry_msgs trajectory_msgs roscpp roslib tf wam_msgs rosbag biotac_sensors cheetah_lib xenomai_ros hardware_interface terse_roscpp controller_manager control_toolbox rospy urdf angles kdl_parser)

catkin_package(
  DEPENDS Barrett
  CATKIN_DEPENDS std_msgs std_srvs sensor_msgs geometry_msgs trajectory_msgs roscpp tf wam_msgs roslib rosbag biotac_sensors cheetah_lib hardware_interface terse_roscpp controller_manager control_toolbox rospy urdf angles kdl_parser   
  INCLUDE_DIRS 
  LIBRARIES 
)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include ${catkin_INCLUDE_DIRS})
find_package(Barrett REQUIRED)
link_directories(${BARRETT_LIB_DIRS} src)
include_directories(${BARRETT_INCLUDE_DIRS})
add_definitions(${BARRETT_DEFINITIONS})




#add_executable(wam_node src/wam_node.cpp src/cheetah.c src/BCbiotac.c)
#target_link_libraries(wam_node ${BARRETT_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(wam_node ${catkin_EXPORTED_TARGETS} wam_msgs_gencpp)

#add_executable(wam_node_1 src/wam_node_1.cpp src/cheetah.c src/BCbiotac.c)
#target_link_libraries(wam_node_1 ${BARRETT_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(wam_node_1 ${catkin_EXPORTED_TARGETS} wam_msgs_gencpp)

#add_executable(wam_node_r src/wam_node_r.cpp src/cheetah.c src/BCbiotac.c)
#target_link_libraries(wam_node_r ${BARRETT_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(wam_node_r ${catkin_EXPORTED_TARGETS} wam_msgs_gencpp)

#add_executable(wam_node_r_edited src/wam_node_r_edited.cpp)
#target_link_libraries(wam_node_r_edited ${BARRETT_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(wam_node_r_edited ${catkin_EXPORTED_TARGETS} wam_msgs_gencpp)

#add_executable(wam_node_l_edited src/wam_node_l_edited.cpp)
#target_link_libraries(wam_node_l_edited ${BARRETT_LIBRARIES} ${catkin_LIBRARIES} biotacpub)
#add_dependencies(wam_node_l_edited ${catkin_EXPORTED_TARGETS} wam_msgs_gencpp)

add_executable(wam_node_l src/wam_node_l.cpp)
target_link_libraries(wam_node_l ${catkin_LIBRARIES} ${BARRETT_LIBRARIES} biotacpub)
add_dependencies(wam_node_l ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} wam_msgs_gencpp)

