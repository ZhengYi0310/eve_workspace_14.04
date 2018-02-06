/*************************************************************************
	> File Name: task_recorder_node.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 27 Jun 2017 11:40:03 AM PDT
 ************************************************************************/

// system includes 
// ros includes 
#include <ros/ros.h>

// local includes 
#include <usc_utilities/assert.h>

#include <task_recorder/joint_states_recorder.h>
#include <task_recorder/pose_recorder.h>
#include <task_recorder/tf_recorder.h>
#include <task_recorder/arm_cartesian_state_recorder.h>
#include <task_recorder/biotac_state_recorder.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TaskRecorder");
    ros::NodeHandle node_handle("~");

    task_recorder::JointStatesRecorder joint_states_recorder(node_handle);
    ROS_VERIFY(joint_states_recorder.initialize());

    //task_recorder::PoseRecorder pose_recorder(node_handle);
    //std::string pose_topic_name = std::string("/pose_r");
    //ROS_VERIFY(pose_recorder.initialize(pose_topic_name));

    task_recorder::CartesianStateRecorder arm_cartesian_state_recorder(node_handle);
    ROS_VERIFY(arm_cartesian_state_recorder.initialize());

    task_recorder::BioTacStateRecorder biotac_state_recorder(node_handle);
    ROS_VERIFY(biotac_state_recorder.initialize());

    //task_recorder::TFRecorder tf_recorder(node_handle);
    //ROS_VERIFY(tf_recorder.initialize());
    ros::spin();
    return 1;
}

