/*************************************************************************
	> File Name: task_recorder_client.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 19 Jun 2017 05:20:08 PM PDT
 ************************************************************************/

#ifndef _TASK_RECORDER_CLIENT_H
#define _TASK_RECORDER_CLIENT_H

// system includes 
#include <ros/ros.h>
#include <string>

#include <usc_utilities/assert.h>

// local includes 
#include <task_recorder/task_recorder_utilities.h>
#include <task_recorder/StartRecording.h>

namespace task_recorder
{
    template<class MessageType, class StopServiceType>
    class TaskRecorderClient
    {
        public:
            TaskRecorderClient() {};
            virtual ~TaskRecorderClient() {};

            /*!
             * @param node_handle 
             * @param topic_name 
             * @return True on success, otherwise False
             */
            bool initialize(ros::NodeHandle& node_handle,
                            const std::string& topic_name);

            /**
             * @Start recording data
             * @return True on success, otherwise False 
             */
            bool startRecording();

            /**
             * @Stop recording data
             * @param start_time 
             * @param end_time 
             * @param num_samples
             * @return 
             */
            bool stopRecording(const ros::Time& start_time,
                               const ros::Time& end_time,
                               int num_samples)

            /**
             * @Stop recording data 
             * @param start_time
             * @param end_time 
             * @param num_samples
             * @return 
             */
            bool stopRecording(const ros::Time& start_time,
                               const ros::Time& end_time,
                               int num_samples,
                               const std::vector<std::string>& message_names);

            /*!
             */
            std::vector<MessageType> messages_;
            std::vector<ros::Time> times_;
            std::vector<double> data_;
            
        private:

            ros::NodeHandle node_handle_;
            std::string topic_name_;

            ros::ServiceClient start_recording_service_client_;
            ros::ServiceClient stop_recording_service_client_;
    };

    template<class MessageType, class StopServiceType>
    bool TaskRecorderClient<MessageType, StopServiceType>::initialize(ros::NodeHandle& node_handle,
                                                                      const std::string& topic_name)
    {
        node_handle_ = node_handle;

        topic_name_ = topic_name;
        std::string service_name = topic_name;
        ROS_VERIFY(task_recorder::getTopicName(service_name));

        std::string start_recording_service_name = "/TaskRecorder/start_recording_" + service_name;
        std::string stop_recording_service_name = "/TaskRecorder/stop_recording_" + service_name;

        start_recording_service_client_ = node_handle_.serviceClient<task_recorder::StartRecording>(start_recording_service_name);
        stop_recording_service_client_ = node_handle_.serviceClient<StopServiceType>(stop_recording_service_name);

        return true;
    }

    template<class MessageType, class StopServiceType>
    bool TaskRecorderClient<MessageType, StopServiceType>::StartRecording()
    {
        task_recorder::StartRecording::Request start_request;
        task_recorder::StartRecording::Response start_respoonse;

        start_request.topic.clear();
        start_request.topic.push_back(topic_name_);

        bool service_online = false;
        while (ros::ok() && !service_online)
        {
            if (!start_recording_service_client_.waitForExistence(ros::Duration(1.0)))
            {
                ROS_WARN("Waiting for >%s< ...", start_recording_service_client_.getService().c_str());
            }
            else 
            {
                service_online = true;
            }
        }
        return start_recording_service_client_.call(start_request, start_respoonse);
    }

    template<class MessageType, class StopServiceType>
    bool TaskRecorderClient<MessageType, StopServiceType>::stopRecording(const ros::Time& start_time,
                                                                         const ros::Time& end_time,
                                                                         int num_samples)
    {
        std::vector<std::string> no_message_names;
        return stopRecording(start_time, end_time, num_samples, no_message_names);
    }

    template<class MessageType, class StopServiceType>
    bool TaskRecorderClient<MessageType, StopServiceType>::stopRecording(const ros::Time& start_time,
                                                                         const ros::Time& end_time,
                                                                         int num_samples,
                                                                         const std::vector<std::string>& message_names)
    {
        messages_.clear();

        typename StopServiceType::Request stop_request;
        typename StopServiceType::Response stop_response;
        stop_request.crop_start_time = start_time;
        stop_request.crop_end_time = end_time;
        stop_request.num_samples = num_samples;
        stop_request.message_names = message_names;

        bool service_online = false;
        while (ros::ok() && !service_online)
        {
            if (!stop_recording_service_client_.waitForExistence(ros::Duration(1.0)))
            {
                ROS_WARN("Waiting for >%s< ...", stop_recording_service_client_.getService().c_str());
            }
            else 
            {
                service_online = true;
            }
        }

        ROS_VERIFY(stop_recording_service_client_.call(stop_request, stop_response));
        if (stop_response.return_code != StopServiceType::Response::SERVICE_CALL_SUCCESSFUL)
        {
            ROS_ERROR("Stop recording service >%s< failed with return code >%i<!", stop_recording_service_client_.getService().c_str(), stop_response.return_code);
            return false;
        }

        messages_ = stop_response.filtered_and_cropped_messages;
        times_ = stop_response.times;
        data_ = stop_response.data;
        return true;
    }

}
#endif
