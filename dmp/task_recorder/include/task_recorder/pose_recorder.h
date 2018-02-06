/*************************************************************************
	> File Name: pose_recorder.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 26 Jun 2017 03:40:01 PM PDT
 ************************************************************************/

#ifndef _POSE_RECORDER_H
#define _POSE_RECORDER_H

// system includes 
#include <vector>
#include <string>

// ros includes 
#include <geometry_msgs/PoseStamped.h>

// local includes 
#include <task_recorder/task_recorder.h>
#include <task_recorder/DataSample.h>

namespace task_recorder
{
    class PoseRecorder : public TaskRecorder<geometry_msgs::PoseStamped>
    {
        public:
            PoseRecorder(ros::NodeHandle node_handle);

            /*! Destructor
             */
            virtual ~PoseRecorder() {};

            /*!
             * @return True on success, otherwise False
             */
            bool initialize(const std::string topic_name = std::string("/pose"))
            {
                return TaskRecorder<geometry_msgs::PoseStamped>::initialize(topic_name);
            }

            /*!
             * @param pose
             * @param data_sample
             * @return True on success, otherwise False
             */
            bool transformMsg(const geometry_msgs::PoseStamped& pose, task_recorder::DataSample& data_sample);

            /*!
             * @return
             */
            int getNumSignals() const
            {
                return static_cast<int>(NUM_POSE_VARIABLES);
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
                return std::string("PoseRecorder");
            }

        private: 

            static const unsigned int NUM_POSE_VARIABLES = 7;
            enum
            {
                POSITION_X = 0, POSITION_Y, POSITION_Z, POSITION_QW, POSITION_QX, POSITION_QY, POSITION_QZ
            };

    };
}

#endif
