/*************************************************************************
	> File Name: message_ring_buffer.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 23 Jun 2017 11:14:12 AM PDT
 ************************************************************************/

#ifndef _MESSAGE_RING_BUFFER_H
#define _MESSAGE_RING_BUFFER_H

// system includes 
#include <vector>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <task_recorder/DataSample.h>

// local includes 
#include <task_recorder_utilities/circular_message_buffer.h>

namespace task_recorder_utilities
{
    class MessageRingBuffer
    {
        static const int DEFAULT_RING_BUFFER_SIZE = 200;

        public:
            
            /*!
             */
            MessageRingBuffer(const task_recorder::DataSample& default_data_sample,
                              const int ring_buffer_size = DEFAULT_RING_BUFFER_SIZE);

            virtual ~MessageRingBuffer() {};

            /*!
             * @param data_sample
             * @return Ture on success, otherwise False 
             */
            bool add(const task_recorder::DataSample& data_sample);

            /*!
             * @param time 
             * @param data_sample
             * return True on success, otherwise False 
             */
            bool get(const ros::Time& time, task_recorder::DataSample& data_sample);

        private:

            /*! Constructor must be initialized with default data sample
             */
            MessageRingBuffer();

            /*!
             */
            boost::shared_ptr<CircularMessageBuffer<task_recorder::DataSample> > circular_buffer_;
    };
}
#endif
