cmake_minimum_required(VERSION 2.8.3)
project(wam_dmp_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  angles
  controller_interface
  hardware_interface
  control_msgs
  control_toolbox
  realtime_tools
  urdf
  std_msgs
  geometry_msgs
  sensor_msgs
  usc_utilities
  dynamic_movement_primitive
  rosrt
  robot_info
  pluginlib
  effort_controllers
  message_generation
  kdl_parser
  forward_command_controller
  dynamic_reconfigure
  tf_conversions
  kdljacdot
  roscpp
  kdl_conversions
  eigen_conversions
  
)

find_package(Boost REQUIRED COMPONENTS thread signals system filesystem)
add_definitions(${Boost_DEFINITIONS})
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost REQUIRED COMPONENTS thread signals system filesystem)
add_definitions(${Boost_DEFINITIONS})

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_message_files(DIRECTORY msg FILES
  JointPositionVelocityStamped.msg
  NullspaceTermStamped.msg
  PoseTwistStamped.msg
  Waypoint.msg
  JointControllerState.msg
  RPY.msg
  PoseRPY.msg
  PoseRPYCmd.msg
  ArmState.msg
  SetJointPosMsg.msg
  SetJointGainsMsg.msg
  GoHomeMsg.msg
)

add_service_files(DIRECTORY srv FILES
  PoseRPYCommand.srv
  ImpedanceControllerGains.srv
  GoHome.srv
  GetJointPos.srv
  SetJointPos.srv
  GetJointGains.srv
  SetJointGains.srv
)
generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    control_toolbox
    realtime_tools
    urdf
    std_msgs
    geometry_msgs
    sensor_msgs
	usc_utilities
	dynamic_movement_primitive
	rosrt
    robot_info
    pluginlib
    effort_controllers
    message_runtime
    kdl_parser
    forward_command_controller
    dynamic_reconfigure
    tf_conversions
    kdljacdot
    roscpp
    kdl_conversions
    eigen_conversions
    
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

add_library(${PROJECT_NAME}
  src/joint_position_controller.cpp
  src/dmp_joint_position_controller.cpp
  src/variable_name_map.cpp
  src/cartesian_twist_controller_ik_with_nullspace_optimization.cpp 
  src/dmp_ik_controller.cpp
  src/KinematicChainControllerBase.cpp
  src/joint_impedance_controller.cpp
  src/joint_space_controller.cpp
  src/gravity_compensation.cpp
  src/arm_state_controller.cpp
  src/operational_space_controller.cpp
  src/operational_space_impedance_controller.cpp
  src/operational_space_impedance_spline_controller.cpp
  src/SISE_kalman_filter.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBARIES} ${EIGEN3_LIBRARIES})

add_executable(test_wam_dmp_controller
  test/test_dmp_controller.cpp
)
target_link_libraries(test_wam_dmp_controller ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBARIES} ${EIGEN3_LIBRARIES})
add_dependencies(test_wam_dmp_controller ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} wam_dmp_controller_gencpp)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

