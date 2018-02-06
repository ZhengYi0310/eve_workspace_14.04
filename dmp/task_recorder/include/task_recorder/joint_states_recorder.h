/*************************************************************************
	> File Name: joint_states_recorder.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 16 Jun 2017 10:22:58 PM PDT
 ************************************************************************/

#ifndef _JOINT_STATES_RECORDER_H
#define _JOINT_STATES_RECORDER_H

// system includes 
#include <vector>
#include <string>

// ros includes 
#include <sensor_msgs/JointState.h>
#include <barrett_hw/joint_state.h>

// local includes 
#include <task_recorder/task_recorder.h>
#include <task_recorder/DataSample.h>

namespace task_recorder
{
    class JointStatesRecorder : public TaskRecorder<barrett_hw::joint_state>
    {
        public:
            
            /*!
             */
            JointStatesRecorder(ros::NodeHandle node_handle);
            virtual ~JointStatesRecorder() {};

            
            /*!
             * @return Ture on success, otherwise False
             */
            bool initialize(const std::string topic_name = std::string("/joint_states_rt"))
            {
                return TaskRecorder<barrett_hw::joint_state>::initialize(topic_name);
            }

            /*!
             * @param joint_state
             * @param data_sample
             * @return True on success, otherwise False
             */
            bool transformMsg(const barrett_hw::joint_state& joint_state, task_recorder::DataSample& data_sample);

            /*!
             * @return
             */
            int getNumSignals() const
            {
                return static_cast<int>(POS_VEL_EFF * joint_names_.size() + 1);
            }

            /*!
             * @return
             */
            std::vector<std::string> getNames() const;

            /*!
             * @return
             */
            static std::string getClassName()
            {
                return std::string("JointStatesRecorder");
            }



        private:
            
            static const int POS_VEL_EFF = 3;
            static const int POSITIONS = 0;
            static const int VELOCITIES = 1;
            static const int EFFORTS = 2;

            int num_joint_states_;
            std::vector<std::string> joint_names_;
            std::vector<int> indices_;
    };
}
#endif
