/*************************************************************************
	> File Name: joint_states_recorder.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 16 Jun 2017 10:22:58 PM PDT
 ************************************************************************/

#ifndef _JOINT_STATES_RECORDER_H
#define _JOINT_STATES_RECORDER_H

// system includes 
#include <vector>

// ros includes 
#include <sensor_msgs/JointState.h>

// local includes 
#include <task_recorder/task_recorder.h>
#include <task_recorder/accumulator.h>

#include <task_recorder/StartRecording.h>
#include <task_recorder/StopRecordingJointStates.h>

namespace task_recorder
{
    class JointStatesRecorder : public TaskRecorder<sensor_msgs::JointState, task_recorder::StopRecordingJointStates>
    {
        public:

            JointStatesRecorder() {};
            virtual ~JointStatesRecorder() {};

            /*!
             * @param node_handle
             * @param topic_name
             * @return 
             */
            bool initialize(ros::node_handle& node_handle,
                            const std::string& topic_name);

            /*!
             * @param start_time
             * @param end_time
             * @param movement_duration
             * @param num_samples
             * @param joint_states 
             * @param message_times 
             * @param times 
             * @param data 
             * @return 
             */
            bool filterAndCrop(const ros::Time& start_time,
                               const ros::Time7 end_time,
                               int num_samples,
                               std::vector<sensor_msgs::JointState>& joint_states,
                               std::vector<std::string>& message_names,
                               std::vector<ros::Time>& times,
                               std::vector<double>& data);

            bool transformMessages(sensor_msgs::JointState& joint_state);

            /*!
             * @parm trial_statistics 
             * @return 
             */
            bool getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& accumulated_trial_statistics_vector);

            /*!
             * @param signal_index 
             * @param signal_name 
             */
            void getSignalNames(const int signal_index,
                                std::string& signal_name);

            /*!
             * @param accumulated_trial_statistics 
             */
            void setMessageNames(std::vector<task_recorder::AccumulatedTrialStatistics> accumulated_trial_statistics);

            void getVariables(const sensor_msgs::JointState& message, std::vector<double>& variables);
            void setVariables(sensor_msgs::JointState& message, const std::vector<double>& variables);

        private:

            bool resample(std::vector<sensor_msgs::JointState>& joint_states,
                          const ros::Time& start_time,
                          const ros::Time& end_time,
                          const int num_samples,
                          std::vector<sensor_msgs::JointState>& resampled_joint_states,
                          std::vector<std::string> joint_names);

            Accumulator position_accumulator_;
            Accumulator velocity_accumulator_;
            Accumulator effort_accumulator_;      
    };
}
#endif
