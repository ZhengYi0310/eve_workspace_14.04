/*************************************************************************
	> File Name: message_ring_buffer.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 23 Jun 2017 01:20:07 PM PDT
 ************************************************************************/

// system includes 
// ros includes 
// local includes 
#include <task_recorder_utilities/message_ring_buffer.h>

namespace task_recorder_utilities
{
    MessageRingBuffer::MessageRingBuffer(const task_recorder::DataSample& default_data_sample, 
                                         const int ring_buffer_size)
    {
        circular_buffer_.reset(new CircularMessageBuffer<task_recorder::DataSample>(ring_buffer_size, default_data_sample));
    }

    bool MessageRingBuffer::add(const task_recorder::DataSample& data_sample)
    {
        // error checking 
        if (!circular_buffer_->empty())
        {
            if ((int)circular_buffer_->front().data.size() != (int)data_sample.data.size())
            {
                ROS_ERROR("Size of data vector >%i< needs to be >%i<.", (int)circular_buffer_->front().data.size(), (int)data_sample.data.size());
                return false;
            }
        }

        // fill the buffer if it has been empty
        if(circular_buffer_->size() == 0)
        {
            for (unsigned int i = 0; i < circular_buffer_->capacity(); ++i)
            {
                circular_buffer_->push_back(data_sample);
            }
        }

        circular_buffer_->push_back(data_sample);
        return true;
    }

    bool MessageRingBuffer::get(const ros::Time& time, task_recorder::DataSample& data_sample)
    {
        bool found = false;
        boost::circular_buffer<task_recorder::DataSample>::const_reverse_iterator rci;
        for (rci = (circular_buffer_->cb_).rbegin(); rci != (circular_buffer_->cb_).rend() && !found; ++rci)
        {
            if (rci->header.stamp <= time)
            {
                data_sample = *rci;
                found = true;
            }
        }

        return found;
    }

    
 }

