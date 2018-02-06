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

// ros includes 
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

// local includes 
#include <task_recorder/AccumulatedTrialStatistics.h>

namespace task_recorder
{
    class Accumulator 
    {
        public:
            
            typedef boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::mean, boost::accumulators::tag::variance> > AccumulatorType;
            
            
            Accumulator() : initialized_(false), num_data_traces_(0); num_samples(0) {};
            virtual ~Accumulator();

            /*!
             * @param num_data_traces
             * @param num_samples 
             * @return 
             */
            bool initialize(const int num_data_traces, const int num_samples);

            /*!
             * @param data_trace_index 
             * @param data_trace 
             * @return 
             */
            bool add(const int data_trace_index, std::vector<double>& data_trace);

            /*!
             * @param trial_statistics
             * @return 
             */
            bool getAccumulatedTrialStatistics(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics);
    
        private:
            
            bool initialized_;

            int num_data_traces_;
            int num_samples_;

            std::vector<std::vector<AccumulatorType> > accumulators_;
    };
}
#endif
