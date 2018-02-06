/*************************************************************************
	> File Name: message_buffer.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 23 Jun 2017 01:10:12 PM PDT
 ************************************************************************/

// system includes 
// ros includes 
// local includes 
#include <task_recorder_utilities/message_buffer.h>

namespace task_recorder_utilities
{
    bool MessageBuffer::add(const task_recorder::DataSample& data_sample)
    {
        // error checking 
        if (!data_samples_.empty())
        {
            if (!(int)data_samples_[0].data.size() != (int)data_sample.data.size())
            {
                ROS_ERROR("Size of data vector >%i< needs to match the size sample data vector size >%i< !", (int)data_samples_[0].data.size(), (int)data_sample.data.size());
                return false;
            }
        }

        data_samples_.push_back(data_sample);
        return true;
    }

    bool MessageBuffer::get(const ros::Time& time, task_recorder::DataSample& data_sample)
    {
        bool found = false;
        int index = 0;
        for (index = (int)data_samples_.size() - 1; index >= 0 && !found; index--)
        {
            if (data_samples_[index].header.stamp < time)
            {
                data_sample = data_samples_[index];
                found = true;
            }
        }

        if (found && index > 0)
        {
            data_samples_.erase(data_samples_.begin(), data_samples_.begin() + (index - 1));
        }

        return found;
    }

}
using namespace std;

