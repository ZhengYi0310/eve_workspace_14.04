/*************************************************************************
	> File Name: message_buffer.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 23 Jun 2017 11:02:52 AM PDT
 ************************************************************************/

#ifndef _MESSAGE_BUFFER_H
#define _MESSAGE_BUFFER_H

// system includes 
#include <vector>
#include <ros/ros.h>

// local includes 
#include <task_recorder/DataSample.h>

namespace task_recorder_utilities
{
    class MessageBuffer
    {
        public:

            /*!
             * @param data_sample 
             * @return True on success, otherwise False 
             */
            MessageBuffer() {};
            virtual ~MessageBuffer() {};

            /*!
             * @param data_sample
             * @return True on success, otherwise False 
             */
            bool add(const task_recorder::DataSample& data_sample);

            /*!
             * @param time 
             * @param data_sample 
             * @return True on success, otherwise False 
             */
            bool get (const ros::Time& time, task_recorder::DataSample& data_sample);

        private:
            std::vector<task_recorder::DataSample> data_samples_;
    };
}
#endif
