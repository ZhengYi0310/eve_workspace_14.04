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
#include <task_recorder/accumulator.h>

using namespace boost;
using namespace boost::accumulators 

namespace task_recorder
{
    bool Accumulator::initialize(const int num_data_traces, const int num_samples)
    {
        // if already initialized, check whether num_data_traces and num_samples are correct 
        if (initialized_)
        {
            ROS_ASSERT(num_data_traces == num_data_traces_);
            ROS_ASSERT(num_samples == num_samples_);
            return true;
        }

        num_data_traces_ = num_data_traces;
        num_samples_ = num_samples;

        std::vector<AccumulatorType> accumulator_vec;
        accumulator_vec.resize(num_samples_);
        accumulators_.resize(num_data_traces_, accumulator_vec);

        return (initialized_ = true);
    }

    bool Accumulator::add(const int data_trace_index, std::vector<double>& data_trace)
    {
        ROS_ASSERT(initialized_);
        ROS_ASSERT((data_trace_index >= 0) && (data_trace_index < num_data_traces_));
        ROS_WARN_COND(num_samples_ != static_cast<int> data_trace.size(), "num_samples_ (%i) == (%i) static_cast<int>(data_trace.size())", num_samples_, static_cast<int>(data_trace.size()));
        ROS_ASSERT(num_samples_ == static_cast<int>(data_trace.size()));

        for (int i = 0; i < data_trace.size(); i++)
        {
            accumulators_[data_trace_index][i](data_trace[i]);
        }

        return true;
    }

    bool Accumulator::getAccumulatedTrialStatistics(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics);
    {
        accumulated_trial_statistics.clear();

        for (int i = 0; i < num_samples_; i++)
        {
            task_recorder::AccumulatedTrialStatistics accumulated_trial_statistic;
            accumulated_trial_statistic.mean.resize(num_data_traces_);
            accumulated_trial_statistic.variance.resize(num_data_traces_);

            for (int j = 0; j < num_data_traces_; i++)
            {
                accumulated_trial_statistic.count = count(accumulators_[j][i]);
                accumulated_trial_statistic.mean[j] = mean(accumulators_[j][i]);
                accumulated_trial_statistic.variance[j] = mean(accumulators_[j][i]);
            }
            accumulated_trial_statistics.push_back(accumulated_trial_statistic);
        }

        return true;
    }
}

