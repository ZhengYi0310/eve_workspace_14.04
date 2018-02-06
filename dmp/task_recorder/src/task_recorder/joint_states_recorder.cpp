/*************************************************************************
	> File Name: joint_states_recorder.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 20 Jun 2017 10:28:19 AM PDT
 ************************************************************************/
// system includes 

// ros includes 
#include <usc_utilities/bspline.h>
#include <usc_utilities/assert.h>

// local includes 
#include <task_recorder/joint_states_recorder.h>
#include <task_recorder_utilities/task_recorder_utilities.h>

namespace task_recorder
{
    JointStatesRecorder::JointStatesRecorder(ros::NodeHandle node_handle) : TaskRecorder<barrett_hw::joint_state>(node_handle), num_joint_states_(0)
    {
        ROS_VERIFY(usc_utilities::read(node_handle, "joint_names", joint_names_));
        ROS_DEBUG("Starting joint states recorder for joint:");
        for (int i = 0; i < (int)joint_names_.size(); ++i)
        {
            ROS_DEBUG("%i) %s", i+1, joint_names_[i].c_str());
        }
    }

    bool JointStatesRecorder::transformMsg(const barrett_hw::joint_state& joint_state,
                                           task_recorder::DataSample& data_sample)
    {
        if(first_time_)
        {
            ROS_VERIFY(task_recorder_utilities::getIndices(joint_state.name, joint_names_, indices_));
            num_joint_states_ = (int)joint_names_.size();
        }
        data_sample.header = joint_state.header;

        ROS_ASSERT((int)data_sample.data.size() == (num_joint_states_ * POS_VEL_EFF + 1));
        // positions, velicities, and acceleration
        for (int i = 0; i < num_joint_states_; ++i)
        {
            data_sample.data[(i * POS_VEL_EFF) + POSITIONS] = joint_state.position[indices_[i]];
            data_sample.data[(i * POS_VEL_EFF) + VELOCITIES] = joint_state.velocity[indices_[i]];
            data_sample.data[(i * POS_VEL_EFF) + EFFORTS] = joint_state.effort[indices_[i]];
        }
        data_sample.data.back() = joint_state.header_from_controller_start.toSec();
        return true;
    }

    std::vector<std::string> JointStatesRecorder::getNames() const
    {
        // ROS_ASSERT_MSG(initialized_, "JointStatesRecorder is not initialize.");
        std::vector<std::string> names;
        const int num_joint_states = (int)joint_names_.size();
        for (int i = 0; i < num_joint_states; ++i)
        {
            names.push_back(joint_names_[i] + std::string("_pos"));
            names.push_back(joint_names_[i] + std::string("_vel"));
            names.push_back(joint_names_[i] + std::string("_eff"));
        }
        names.push_back("header_from_controller_start");
        return names;
    }
}

