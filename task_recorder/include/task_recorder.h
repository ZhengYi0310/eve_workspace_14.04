/*************************************************************************
	> File Name: task_recorder.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 16 Jun 2017 10:24:43 PM PDT
 ************************************************************************/

#ifndef _TASK_RECORDER_H
#define _TASK_RECORDER_H

// system includes 
#include <vector>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// ros includes 
#include <ros/ros.h>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/bspline.h>

// local includes 
#include <task_recorder/task_recorder_io.h>
#include <task_recorder/task_recorder_utilities.h>
#include <task_recorder/StartRecording.h>
#include <task_recorder/accumulator.h>
#include <task_recorder/AccumulatedTrialStatistics.h>

namespace task_recorder 
{
    template <class MessageType, class StopServiceType>
    class TaskRecorder 
    {
        public:
            
            typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;
            typedef boost::shared_ptr<StopServiceType const> StopServiceTypeConstPtr;

            TaskRecorder() : initialized_(false), is_recording_(false) {};
            virtual ~TaskRecorder() {};

            /*!
             * @param node_handle 
             * @param topic_name 
             * return True on success, otherwise False 
             */
            bool initializeBase(ros::NodeHandle& node_handle,
                                const std::string& topic_name);

            void startRecording();

            void stopRecording();

            /*!
             * @param request 
             * @param response 
             * @return True on success, otherwise False 
             */
            bool startRecording(task_recorder::StartRecording::Request& request,
                                task_recorder::StartRecording::Response& response);

            /*!
             * @param request 
             * @param response 
             * @return True on success, otherwise False 
             */
            bool stopRecording(typename StopServiceType::Request& request,
                               typename StopServiceType::Response& response);

        protected:

            bool initialized_;
            bool is_recording_;
            
            TaskRecorderIO<MessageType> recorder_io_;

            /*!
             * @param start_time
             * @param end_time 
             * @param num_samples
             * @param filter_and_cropped_messages
             * @param message_names
             * @param times
             * @param data 
             * @return True on success, otherwise False 
             */
            virtual bool filterAndCrop(const ros::Time& start_time,
                                       const ros::Time& end_time,
                                       int num_samples,
                                       std::vector<MessageType>& filtered_and_cropped_messages,
                                       std::vector<std::string>& message_names,
                                       std::vector<ros::Time>& times,
                                       std::vector<double>& data);

            /*!
             * @param message 
             * @return True on success, otherwise False 
             */
            virtual bool transformMessage(MessageType& message);

            /*!
             * @pram vector_of_accumulated_statistics
             * @return True on success, otherwise False 
             */
            virtual bool getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& accumulated_trial_statistics_vector);

            /*!
             * @param accumulated_trial_statistics 
             */
            virtual void setMessageNames(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics) = 0;

            /*!
             * @param signal_index
             * @param singal_name 
             */
            virtual void getSignalNames(const int signal_index,
                                        std::string& signal_name) = 0;
            virtual void getVariables(const MessageType& message, std::vector<double>& variables) = 0;
            virtual void setVariables(MessageType& message, const std::vector<double>& variables) = 0;
            virtual int getNumVariables() = 0;

        private:

            ros::ServiceServer start_recording_service_server_;
            ros::ServiceServer stop_recording_service_server_;

            ros::Subscriber subscriber_;

            boost::mutex mutex_;
            bool logging_;

            task_recorder::Accumulator accumulator_;

            void recordMessageCallback(const MessageTypeConstPtr message);

            bool resample(std::vector<MessageType>& messages,
                          const ros::Time& start_time,
                          const ros::Time& end_time,
                          const int num_samples,
                          std::vector<MessageType>& resampled_messages,
                          bool use_bspline=false);
    };

    template<class MessageType, class StopServiceType>
    bool TaskRecorder<MessageType, StopServiceType>::initializeBase(ros::NodeHandle& node_handle,
                                                                    const std::string& topic_name)
    {
        if(!recorder_io_.initialize(node_handle, topic_name))
        {
            ROS_ERROR("Could not initialize the base of the task recorder.");
            return (initialized_ = false);
        }

        std::string name = topic_name;
        if(!getTopicName(name))
        {
            ROS_ERROR("Could not obtian topic name >%s<. Could not initialize the base of the task recorder.", name.c_str());
            return (initialized_ = false);
        }
        std::string start_recording_service_name = std::string("start_recording_" + name);
        std::string stop_recording_service_name = std::string("stop_recording_" + name);

        start_recording_service_server_ = recorder_io_.node_handle_.advertiseService(start_recording_service_name, &TaskRecorder<MessageType, StopServiceType>::startRecording, this);
        stop_recording_service_server_ = recorder_io_.node_handle_.advertiseService(stop_recording_service_name, &TaskRecorder<MessageType, StopServiceType>::stopRecording, this);

        return (initialized_ = true);
    }

    template<class MessageType, class StopServiceType>
    void TaskRecorder<MessageType, StopServiceType>::recordMessageCallback(const MessageTypeConstPtr message)
    {
        mutex_.lock() ;
        if (logging_)
        {
            MessageType msg = *message;
            transformMessage(msg);
            recorder_io_.messages_.push_back(msg);
        }
        mutex_.unlock();
    }

    template<class MessageType, class StopServiceType>
    void TaskRecorder<MessageType, StopServiceType>::startRecording()
    {
        ROS_INFO("Start recording topic named >%s<.", recorder_io_.topic_name_.c_str());
        subscriber_ = recorder_io_.node_handle_.subscribe(recorder_io_.topic_name_, 10000, &TaskRecorder<MessageType, StopServiceType>::recordMessageCallback, this);
        mutex_.lock();
        logging_ = true;
        recorder_io_.messages_.clear();
        mutex_.unlock();
    }

    template<class MessageType, class StopServiceType>
    void TaskRecorder<MessageType, StopServiceType>::stopRecording()
    {
        ROS_INFO("Stop recording topic named >%s<.", recorder_io_.topic_name_.c_str());
        mutex_.lock();
        logging_ = false;
        mutex_.unlock();
        subscriber_.shutdown();
    }

    template<class MessageType, class StopServiceType>
    bool TaskRecorder<MessageType, StopServiceType>::startRecording(task_recorder::StartRecording::Request& request, task_recorder::StartRecording::Response& response)
    {
        startRecording();
        recorder_io_.setId(request.id);
        response.return_code = task_recorder::StartRecording::SERVICE_CALL_SUCCESSFUL;
        return true;
    }

    template<class MessageType, class StopServiceType>
    bool TaskRecorder<MessageType, StopServiceType>::stopRecording(typename StopServiceType::Request& request, typename StopServiceType::Response& response)
    {
        stopRecording();
        std::vector<MessageType> filtered_and_cropped_messages;
        std::vector<ros::Time> times;
        std::vector<double> data;

        if (!filterAndCrop(request.crop_start_time, request.crop_end_time, request.num_samples, filtered_and_cropped_messages, request.message_names, times, data))
        {
            ROS_ERROR("Could not filter and crop messages.");
            response.return_code = StopServiceType::Response::SERVICE_CALL_FAILED;
            return true;
        }

        // write out data files 
        boost::thread(boost::bind(&TaskRecorderIO<MessageType>::writeRecordedData, recorder_io_));

        // write out statistics files 
        std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> > accumulated_trial_statistics_vec;
        ROS_VERIFY(getAccumulatedTrialStatistics(accumulated_trial_statistics_vec));
        boost::thread(boost::bind(&TaskRecorderIO<MessageType>::writeStatistics, recorder_io_, accumulated_trial_statistics_vec));

        response.filtered_and_cropped_messages = filter_and_cropped_messages;
        response.times = times;
        response.data = data;
        response.return_code = StopServiceType::Response::SERVICE_CALL_SUCCESSFUL;
        return true;
    }

    template<class MessageType, class StopServiceType>
    bool TaskRecorder<MessageType, StopServiceType>::filterAndCrop(const ros::Time& start_time,
                                                                   const ros::Time& end_time,
                                                                   int num_samples,
                                                                   std::vector<MessageType>& filtered_and_cropped_messages,
                                                                   std::vector<std::string>& message_names,
                                                                   std::vector<ros::Time>& times,
                                                                   std::vector<double>& data)
    {
        int num_messages = recorder_io_.messages_.size();
        if (num_messages == 0)
        {
            ROS_ERROR("Zero messages have been logged.");
            return false;
        }

        // figure out when out data starts and ends 
        ros::Time our_start_time = recorder_io_.messages_[0].header.stamp;
        ros::Time our_end_time = recorder_io_.messages_[num_messages - 1].header.stamp;
        int index = 0;
        while (our_end_time.toSec() < 1e-6)
        {
            index++;
            our_end_time = recorder_io_.messages_[num_messages - (1 + index)].header.stamp;
        }

        if (our_start_time > start_time || our_end_time < end_time)
        {
            ROS_ERROR("Requested times have not been recorded!");
            ROS_ERROR_STREAM("Recorded start and end times : " << our_start_time << " to " << our_end_time);
            ROS_ERROR_STREAM("Requested start and end times: " << start_time << " to " << end_time);
            return false;
        }

        // fit bspline and resample the position and effort trajectories and compute the velocities 
        ROS_VERIFY(resample(recorder_io_.messages_, start_time, end_time, num_samples, filtered_and_cropped_messages, true));
        ROS_ASSERT(static_cast<int>(filtered_and_cropped_messages.size()) == num_samples);

        recorder_io_.messages_ = filtered_and_cropped_messages;
        return true;
    }

    template<class MessageType, class StopServiceType>
    bool TaskRecorder<MessageType, StopServiceType>::resample(std::vector<MessageType>& messages,
                                                              const ros::Time& start_time,
                                                              const ros::Time& end_time,
                                                              const int num_samples,
                                                              std::vector<MessageType>& resampled_messages,
                                                              bool use_bspline)
    {
        int num_vars = getNumVariables();
        ROS_VERIFY(!messages.empty());

        ROS_VERIFY(removeDuplicates<MessageType>(messages));
        ROS_VERIFY(recorder_io_.writeRawData());

        int num_messages = static_cast<int>(messages.size());

        double dts[num_messages - 1];
        double mean_dt = 0.0;

        std::vector<double> input_vector(num_messages);
        input_vector[0] = messages[0].header.stamp.toSec();
        for (int i = 0; i < num_messages - 1; i++)
        {
            dts[i] = messages[i + 1].header.stamp.toSec() - messages[i].header.stamp.toSec();
            mean_dt += dts[i];
            input_vector[i + 1] = input_vector[i] + dts[i];
        }
        mean_dt /= static_cast<double>(num_messages - 1);

        ros::Duration interval = static_cast<ros::Duration> (end_time - start_time) * (1.0 / double(num_samples - 1));

        double wave_length = interval.toSec() * static_cast<double>(2.0);

        resampled_messages.clear();
        std::vector<double> input_querry(num_samples);
        for (int i = 0; i < num_samples; i++)
        {
            MessageType msg = messages[0];
            msg.header.seq = i;
            msg.header.stamp = static_cast<ros::Time>(start_time.toSec() + i * interval.toSec());
        
            input_querry[i] = msg.header.stamp.toSec();
            resampled_messages.push_back(msg);
        }

        std::vector<std::vector<double> > variables;
        std::vector<std::vector<double> > variables_resampled;

        variables.resize(num_vars);
        variables_resampled.resize(num_vars);

        std::vector<double> temp_vars;
        temp_vars.resize(num_vars, 0.0);

        for (int j = 0; j < num_messages; j++)
        {
            getVariables(messages[j], temp_vars);
            for (int i = 0 ; i < num_vars; i++)
            {
                variables[i].push_back(temp_vars[i]);
            }
        }

        for (int i = 0; i < num_vars; i++)
        {
            if (use_bspline)
            {
                if (!usc_utilities::resample(input_vector, variables[i], wave_length, input_querry, variables_resampled[i], false))
                {
                    ROS_ERROR("Could not resample vairables, splining failed.");
                    return false;
                }
            }
            else 
            {
                ROS_VERIFY(usc_utilities::resampleLinear(input_vector, variables[i], input_querry, variables_resampled[i]));
            }
        }

        for (int j = 0; j < num_samples; j++)
        {
            for (int i = 0; i < num_vars; i++)
            {
                temp_vars[i] = variables_resampled[i][j];
                setVariables(resampled_messages[j], temp_vars);
            }
        }

        return true;
    }

    template<class MessageType, class StopServiceType>
    bool TaskRecorder<MessageType, StopServiceType>::getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& accumulated_trial_statistics_vector)
    {
        accumulated_trial_statistics_vector.clear();
        std::vector<task_recorder::AccumulatedTrialStatistics> accumulated_trial_statistics;
        ROS_VERIFY(accumulator_.getAccumulatedTrialStatistics(accumulated_trial_statistics));
        setMessageNames(accumulated_trial_statistics);
        accumulated_trial_statistics_vector.push_back(accumulated_trial_statistics);
        return true;
    }

    template<class MessageType, class StopServiceType>
    bool TaskRecorder<MessageType, StopServiceType>::transformMessages(MessageType& message)
    {
        return true;
    }



}
#endif
