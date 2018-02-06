cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(task_recorder)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rosbag
  rospy
  tf
  filters
  message_filters
  sensor_msgs
  std_msgs
  geometry_msgs
  angles
  usc_utilities
  message_generation
  dynamic_movement_primitive 
  rosbag
  barrett_hw
)


#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIRS})
#add_definitions(${EIGEN3_DEFINITIONS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS thread signals system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
add_definitions(${Boost_DEFINITIONS})


include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
 DIRECTORY msg
 FILES
 AccumulatedTrialStatistics.msg
 BinaryLabel.msg
 CostLabel.msg
 DataSample.msg
 DataSampleLabel.msg
 Description.msg
 Notification.msg
 TaskRecorderSpecification.msg
)

add_service_files(
 DIRECTORY srv
 FILES 
 Accumulate.srv
 AddDataSamples.srv
 DetectedEvents.srv
 GetDataSample.srv
 GetInfo.srv
 InterruptRecording.srv
 ReadDataSamples.srv
 SetLabel.srv
 SetLastTrialIds.srv
 SetupDetector.srv
 StartRecording.srv
 StartStreaming.srv
 StopRecording.srv
 StopStreaming.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs) 





catkin_package(INCLUDE_DIRS include
               LIBRARIES task_recorder_utilities ${PROJECT_NAME} task_recorder_manager task_recorder_manager_client
	       CATKIN_DEPENDS roscpp rosbag tf filters message_filters sensor_msgs std_msgs geometry_msgs angles usc_utilities message_runtime dynamic_movement_primitive rosbag barrett_hw
               DEPENDS Eigen Orocos_kdl Boost)

add_library(task_recorder_utilities
	src/task_recorder_utilities/accumulator.cpp
	src/task_recorder_utilities/message_buffer.cpp
        src/task_recorder_utilities/message_ring_buffer.cpp
)
target_link_libraries(task_recorder_utilities ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES} ${Boost_THREAD_LIBRARIES})

add_library(${PROJECT_NAME}
	src/task_recorder/joint_states_recorder.cpp
	src/task_recorder/pose_recorder.cpp
	src/task_recorder/tf_recorder.cpp
        src/task_recorder/arm_cartesian_state_recorder.cpp
	src/task_recorder/biotac_state_recorder.cpp
	
)
target_link_libraries(${PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES} ${Boost_THREAD_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
add_executable(task_recorder_node
  src/task_recorder/task_recorder_node.cpp
)
target_link_libraries(task_recorder_node ${PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES} ${Boost_THREAD_LIBRARIES} task_recorder_utilities)
add_dependencies(task_recorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(task_recorder_manager
  src/task_recorder/task_recorder_manager.cpp
)
target_link_libraries(task_recorder_manager ${PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES} ${Boost_THREAD_LIBRARIES})

add_executable(task_recorder_manager_node
  src/task_recorder/task_recorder_manager_node.cpp
)
target_link_libraries(task_recorder_manager_node task_recorder_manager ${PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES} ${Boost_THREAD_LIBRARIES} task_recorder_utilities)
add_dependencies(task_recorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(task_recorder_manager_client
  src/task_recorder/task_recorder_manager_client.cpp
)
target_link_libraries(task_recorder_manager_client task_recorder_manager ${PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES} ${Boost_THREAD_LIBRARIES})

add_executable(test_task_recorder_manager_node
  test/test_task_recorder_manager.cpp
)
target_link_libraries(test_task_recorder_manager_node task_recorder_manager_client task_recorder_manager ${PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES} ${Boost_THREAD_LIBRARIES})

add_executable(test_task_recorder_manager_node
  test/test_task_recorder_manager.cpp
)
target_link_libraries(test_task_recorder_manager_node task_recorder_manager_client task_recorder_manager ${PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES} ${Boost_THREAD_LIBRARIES})

add_executable(test_task_recorder_node
  test/test_task_recorder_node.cpp
)
target_link_libraries(test_task_recorder_node task_recorder_manager_client task_recorder_manager ${PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES} ${Boost_THREAD_LIBRARIES})



#add_executable(usc_utilities_test
#	test/asserts_enabled_test.cpp
#	test/asserts_disabled_test.cpp
#	test/param_server_test.cpp
#	test/accumulator_test.cpp
#	test/test_main.cpp
#)
#add_dependencies(tests usc_utilities_test)
#add_rostest(launch/usc_utilities_test.test)
#target_link_libraries(usc_utilities_test usc_utilities)
#target_link_libraries(usc_utilities_test gtest)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
