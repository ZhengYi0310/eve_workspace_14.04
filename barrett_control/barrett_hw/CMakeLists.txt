# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(barrett_hw)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin COMPONENTS hardware_interface controller_interface 
  barrett_model kdl_urdf_tools kdl_parser kdl_conversions terse_roscpp controller_manager biotac_sensors
  control_toolbox rospy xenomai_ros realtime_tools pluginlib visualization_msgs cmake_modules)

find_package(barrett)
find_package(orocos_kdl REQUIRED)
include_directories(
  ${orocos_kdl_INCLUDE_DIRS}
)


if(Xenomai_FOUND)
  add_message_files(DIRECTORY msg FILES
  		arm_cartesian_state.msg
        robot_cartesian_state.msg
		robot_states_tactile.msg
  		robot_states_no_tactile.msg
        joint_state.msg
  )
  generate_messages(DEPENDENCIES
  biotac_sensors
  geometry_msgs
  std_msgs
  visualization_msgs
  )

  include_directories(include ${catkin_INCLUDE_DIRS})

  # CATKIN_MIGRATION: removed during catkin migration
  # include(${rosbuild_xenomai_PACKAGE_PATH}/rosbuild_xenomai.cmake)

  find_package(Eigen REQUIRED)
  include_directories(${EIGEN_INCLUDE_DIRS})
  add_definitions(${EIGEN_DEFINITIONS})

  # Barrett
  include_directories(${BARRETT_INCLUDE_DIRS})
  link_directories(${BARRETT_LIBRARY_DIRS})
  add_definitions(${BARRETT_DEFINITIONS})

  # Add xenomai flags
  add_xenomai_flags()

  add_library(biotac_state_controller src/biotac_state_controller.cpp)
  target_link_libraries(biotac_state_controller xenomai native rtdm ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

  add_library(arm_cartesian_state_controller src/arm_cartesian_state_controller.cpp )
  target_link_libraries(arm_cartesian_state_controller xenomai native rtdm ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

  add_library(robot_states_publisher src/robot_states_publisher.cpp )
  target_link_libraries(robot_states_publisher xenomai native rtdm ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

  add_library(wam_server_lib src/wam_server_copy.cpp)
  target_link_libraries(wam_server_lib xenomai native rtdm ${BARRETT_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
  
  add_library(base_robot_hw src/base_robot_hw.cpp)
  target_link_libraries(base_robot_hw xenomai native rtdm ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

  #add_definitions(-D__XENO__)
  add_executable(wam_server src/wam_server_copy.cpp)
  target_link_libraries(wam_server xenomai native rtdm ${BARRETT_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
  add_dependencies(wam_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  add_executable(wam_server_ll src/wam_server_ll.cpp)
  target_link_libraries(wam_server_ll xenomai native rtdm ${BARRETT_LIBRARIES} ${catkin_LIBRARIES})
  add_dependencies(wam_server_ll ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  add_executable(check_kdl_parser src/check_kdl_parser.cpp )
  target_link_libraries(check_kdl_parser wam_server_lib xenomai native rtdm ${BARRETT_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
  add_dependencies(check_kdl_parser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  ## Generate added messages and services with any dependencies listed here
  # TODO: fill in what other packages will need to use this package
  ## DEPENDS: system dependencies of this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## INCLUDE_DIRS: 
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  catkin_package(
      DEPENDS hardware_interface controller_interface barrett barrett_model kdl_urdf_tools kdl_parser kdl_conversions terse_roscpp controller_manager control_toolbox rospy biotac_sensors realtime_tools pluginlib orocos_kdl visualization_msgs cmake_modules
      CATKIN_DEPENDS # TODO
      INCLUDE_DIRS include
      LIBRARIES wam_server_lib
  )

  install(TARGETS wam_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

endif()
