/*************************************************************************
	> File Name: accumulator.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 16 Jun 2017 09:15:21 PM PDT
 ************************************************************************/

// system includes 
#include <algorithm>

// ros includes 
#include <ros/ros.h>
#include <usc_utilities/assert.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>

// local includes 
#include <task_recorder_utilities/accumulator.h>

using namespace boost;
using namespace boost::accumulators; 

namespace task_recorder_utilities
{
    void Accumulator::clear()
    {
        ROS_DEBUG("Clearning accumulator.");
        accumulators_.setZero(num_data_traces_, num_data_samples_);
        buffer_.setZero(num_data_traces_, num_data_samples_);
        counter_ = 0;
    }

    void Accumulator::getHeaders(const std::vector<task_recorder::DataSample>& data_samples,
                                 std::vector<std_msgs::Header>& headers)
    {
        headers.clear();
        for (int i = 0; i < (int)data_samples.size(); i++)
        {
            headers.push_back(data_samples[i].header);
        }
    }
    
    bool Accumulator::accumulate(std::vector<task_recorder::DataSample>& data_samples)
    {
        ROS_ASSERT_MSG(!data_samples.empty(), "Data samples are empty. Cannot accumulate!");
        ROS_ASSERT_MSG(!data_samples[0].names.empty(), "Data samples do not contain any variable name. Cannot accumulate!");

        if ((counter_ == 0) || ((accumulators_.rows() != num_data_traces_) || (accumulators_.cols() != num_data_samples_)))
        {
            num_data_traces_ = (int)data_samples[0].names.size();
            num_data_samples_ = (int)data_samples.size();

            ROS_DEBUG("Allocating matrix of size >%i< x >%i< !", num_data_traces_, num_data_samples_);
            accumulators_ = Eigen::MatrixXd::Zero(num_data_traces_, num_data_samples_);
            buffer_ = Eigen::MatrixXd::Zero(num_data_traces_, num_data_samples_);
        }

        ROS_ASSERT_MSG((int)accumulators_.rows() == num_data_traces_, "Number of data traced of the accumulator >%i< does not match number of data traces >%i<.", (int)accumulators_.rows(), num_data_traces_);
        ROS_ASSERT_MSG((int)accumulators_.cols() == num_data_samples_, "Number of data samples of the accumulator >%i< does not match number of data samples >%i<.", (int)accumulators_.cols(), num_data_samples_);
        ROS_DEBUG("Accumulating >%i< data samples with >%i< data traces each.", num_data_samples_, num_data_traces_);

        // more error checking 
        std::vector<ros::Time> header_times;
        for (int i = 0; i < num_data_samples_; i++)
        {
            if (i == 0)
            {
                header_times.push_back(data_samples[i].header.stamp);
            }

            ROS_ASSERT_MSG(num_data_traces_ == (int)data_samples[i].names.size(),
                           "Number of names in data sample >%i< should be >%i<, but is >%i<.",i, num_data_traces_, (int)data_samples[i].names.size());

            ROS_ASSERT_MSG(num_data_traces_ == (int)data_samples[i].data.size(),
                           "Number of data points in data sample >%i< should be >%i<, but is >%i<.", i, num_data_traces_, (int)data_samples[i].data.size());
        }

        for (int i = 0; i < num_data_samples_; i++)
        {
            double* ptr = &(data_samples[i].data[0]);
            Eigen::Map<Eigen::VectorXd> temp_vec(ptr, (int)data_samples[i].data.size());
            buffer_.block(0, i, num_data_traces_, 1) = temp_vec;
            //buffer_.block(0, i, num_data_traces_, 1)= Eigen::VectorXd::Map(&data_samples[i].data[0], data_samples[i].data.size());
        }
        accumulators_ += buffer_;
        counter_++;
        return true;
    }

    bool Accumulator::getAccumulatedTrialStatistics(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics)
    {
        accumulated_trial_statistics.clear();
        ROS_INFO("Computing >%i< data traces with >%i< data samples each accumulated over >%i< trials.", num_data_traces_, num_data_samples_, counter_);
        for (int i = 0; i < num_data_samples_; i++)
        {
            task_recorder::AccumulatedTrialStatistics accumulated_trial_statistic;
            accumulated_trial_statistic.mean.resize(num_data_traces_);
            accumulated_trial_statistic.variance.resize(num_data_traces_);

            for (int j = 0; j < num_data_traces_; i++)
            {
                accumulated_trial_statistic.count = counter_;
                accumulated_trial_statistic.mean[j] = accumulators_(j, i) / static_cast<int>(counter_);
                accumulated_trial_statistic.variance[j] = 0;
            }
            accumulated_trial_statistics.push_back(accumulated_trial_statistic);
        }

        return true;
    }
}

