/*************************************************************************
	> File Name: arm_cartesian_state_recorder.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 19 Jul 2017 04:17:12 PM PDT
 ************************************************************************/

#ifndef _ARM_CARTESIAN_STATE_RECORDER_H
#define _ARM_CARTESIAN_STATE_RECORDER_H

// system includes 
#include <barrett_hw/arm_cartesian_state.h>

// local includes 
#include <task_recorder/task_recorder.h>
#include <task_recorder/DataSample.h>

namespace task_recorder
{
    class CartesianStateRecorder : public TaskRecorder<barrett_hw::arm_cartesian_state>
    {
        public:
            CartesianStateRecorder(ros::NodeHandle node_handle);

            /*! Destructor
             */
            virtual ~CartesianStateRecorder() {};

            /*!
             * @return Ture on success, otherwise False 
             */
            bool initialize(const std::string topic_name = std::string("/cartesian_pose"))
            {
                return TaskRecorder<barrett_hw::arm_cartesian_state>::initialize(topic_name);
            }

            /*!
             * @param arm_cartesian_state 
             * @param data_sample 
             * @return True on suceess, otherwise False 
             */
            bool transformMsg(const barrett_hw::arm_cartesian_state& arm_cartesian_state, task_recorder::DataSample& data_sample);

            /*!
             * @return 
             */
            int getNumSignals() const 
            {
                return static_cast<int>(NUM_CARTESIAN_POSE_VARIABLES + 1);
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
                return std::string("CartesianPoseRecorder");
            }

        private:
            
            static const unsigned int NUM_CARTESIAN_POSE_VARIABLES = 13;
            enum
            {
                POSE_X = 0, POSE_Y, POSE_Z, POSE_QW, POSE_QX, POSE_QY, POSE_QZ, TWIST_X, TWIST_Y, TWIST_Z, TWIST_LINX, TWIST_LINY, TWIST_LINZ
            };

            std::string arm_name_;
    };
}
#endif
