
cmake_minimum_required(VERSION 2.8.3)
project(wam_teleop)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_srvs wam_msgs)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(wam_joystick_teleop src/wam_joystick_teleop.cpp)
add_dependencies(wam_joystick_teleop ${catkin_EXPORTED_TARGETS})
target_link_libraries(wam_joystick_teleop ${catkin_LIBRARIES})

catkin_package(
    DEPENDS roscpp sensor_msgs std_srvs joy wam_msgs
)
