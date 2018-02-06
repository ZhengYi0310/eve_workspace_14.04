/*************************************************************************
	> File Name: accumulator.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 16 Jun 2017 06:22:56 PM PDT
 ************************************************************************/

#ifndef _ACCUMULATOR_H
#define _ACCUMULATOR_H

// system includes 
#include <vector>
#include <Eigen/Eigen>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <ros/ros.h>

// local includes 
#include <task_recorder/AccumulatedTrialStatistics.h>
#include <task_recorder/DataSample.h>

namespace task_recorder_utilities
{
    class Accumulator 
    {
        public:
            
            typedef boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::mean, boost::accumulators::tag::variance> > AccumulatorType;
            
            
            Accumulator() : initialized_(false), counter_(0), num_data_traces_(0), num_data_samples_(0) {};
            virtual ~Accumulator();

            /*!
             * @param num_data_traces
             * @param num_samples 
             * @return 
             */
            //bool initialize(const int num_data_traces, const int num_data_samples);

            /*!
             * @param data_samples
             * @param headers
             */
            void getHeaders(const std::vector<task_recorder::DataSample>& data_samples,
                            std::vector<std_msgs::Header>& headers);

            /*!
             */
            void clear();

            /*!
             * @param data_samples
             * @return True on success, otherwise False
             */
            bool accumulate(std::vector<task_recorder::DataSample>& data_samples);

            /*!
             * @param trial_statistics
             * @return 
             */
            bool getAccumulatedTrialStatistics(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics);
    
        private:
            
            bool initialized_;

            int counter_;
            int num_data_traces_;
            int num_data_samples_;

            Eigen::MatrixXd accumulators_;
            Eigen::MatrixXd buffer_;

            //std::vector<std::vector<AccumulatorType> > accumulators_;
    };
}
#endif
