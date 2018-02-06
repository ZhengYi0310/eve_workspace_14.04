/*************************************************************************
	> File Name: arm_cartesian_state_recorder.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 19 Jul 2017 05:15:34 PM PDT
 ************************************************************************/

// ros includes 
#include <usc_utilities/assert.h>

// local includes 
#include <task_recorder/arm_cartesian_state_recorder.h>
#include <task_recorder_utilities/task_recorder_utilities.h>

namespace task_recorder
{
    CartesianStateRecorder::CartesianStateRecorder(ros::NodeHandle node_handle) : TaskRecorder<barrett_hw::arm_cartesian_state>(node_handle)
    {
        //ROS_VERIFY(usc_utilities::read(node_handle, "robot_name", arm_name_));
        //ROS_DEBUG("Starting the cartesian pose reocrder for the arm %s", arm_name_.c_str());
    }

    bool CartesianStateRecorder::transformMsg(const barrett_hw::arm_cartesian_state& arm_cartesian_state, task_recorder::DataSample& data_sample)
    {
        data_sample.header = arm_cartesian_state.header;
        data_sample.header.frame_id = arm_cartesian_state.base_frame;
        data_sample.data[POSE_X] = arm_cartesian_state.Pose.position.x;
        data_sample.data[POSE_Y] = arm_cartesian_state.Pose.position.y;
        data_sample.data[POSE_Z] = arm_cartesian_state.Pose.position.z;
        data_sample.data[POSE_QW] = arm_cartesian_state.Pose.orientation.w;
        data_sample.data[POSE_QX] = arm_cartesian_state.Pose.orientation.x;
        data_sample.data[POSE_QY] = arm_cartesian_state.Pose.orientation.y;
        data_sample.data[POSE_QZ] = arm_cartesian_state.Pose.orientation.z;
        data_sample.data[TWIST_X] = arm_cartesian_state.Twist.linear.x;
        data_sample.data[TWIST_Y] = arm_cartesian_state.Twist.linear.y;
        data_sample.data[TWIST_Z] = arm_cartesian_state.Twist.linear.z;
        data_sample.data[TWIST_LINX] = arm_cartesian_state.Twist.angular.x;
        data_sample.data[TWIST_LINY] = arm_cartesian_state.Twist.angular.y;
        data_sample.data[TWIST_LINZ] = arm_cartesian_state.Twist.angular.z;
        data_sample.data[TWIST_LINZ + 1] = arm_cartesian_state.header_from_controller_start.toSec();

        return true;
    }

    std::vector<std::string> CartesianStateRecorder::getNames() const 
    {
        std::vector<std::string> names;
        names.push_back("POSE_X");
        names.push_back("POSE_Y");
        names.push_back("POSE_Z");
        names.push_back("POSE_QW");
        names.push_back("POSE_QX");
        names.push_back("POSE_QY");
        names.push_back("POSE_QZ");
        names.push_back("TWIST_X");
        names.push_back("TWIST_Y");
        names.push_back("TWIST_Z");
        names.push_back("TWIST_LINX");
        names.push_back("TWIST_LINY");
        names.push_back("TWIST_LINZ");
        names.push_back("header_from_controller_start");
        
        return names;
    }

}

