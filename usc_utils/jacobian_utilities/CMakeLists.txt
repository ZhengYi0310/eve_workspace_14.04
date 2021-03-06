cmake_minimum_required(VERSION 2.4.6)
project(jacobian_utilities)

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

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  robot_info
)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIRS})
#add_definitions(${EIGEN3_DEFINITIONS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS cmake_modules robot_info
               DEPENDS Eigen Eigen3)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} src/jacobian.cpp)


#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

# commented out because it depends on ARM
#rosbuild_add_executable(test_jacobian
#						test/test_jacobian.cpp)
#target_link_libraries(test_jacobian ${PROJECT_NAME})
