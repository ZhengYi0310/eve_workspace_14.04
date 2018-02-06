/*************************************************************************
	> File Name: task_recorder_manager.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 26 Jun 2017 03:51:40 PM PDT
 ************************************************************************/

#ifndef _TASK_RECORDER_MANAGER_H
#define _TASK_RECORDER_MANAGER_H

// system includes 
#include <vector>
#include <map>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

// ros includes 
#include <ros/ros.h>

// local includes 
#include <task_recorder/DataSample.h>

#include <task_recorder/joint_states_recorder.h>
#include <task_recorder/tf_recorder.h>
#include <task_recorder/pose_recorder.h>

#include <task_recorder/task_recorder_client.h>
#include <task_recorder/task_recorder_io.h>
#include <task_recorder/task_recorder.h>

#include <task_recorder/StartStreaming.h>
#include <task_recorder/StopStreaming.h>
#include <task_recorder/StartRecording.h>
#include <task_recorder/StopRecording.h>
#include <task_recorder/InterruptRecording.h>
#include <task_recorder/GetInfo.h>
#include <task_recorder/GetDataSample.h>
#include <task_recorder/AddDataSamples.h>
#include <task_recorder/ReadDataSamples.h>

#include <task_recorder/TaskRecorderSpecification.h>

namespace task_recorder
{
    class TaskRecorderManager
    {
        public:

            TaskRecorderManager(ros::NodeHandle node_handle);
            virtual ~TaskRecorderManager() {};

            /*!
             * @return True on success, False otherwise
             */
            bool initialize();

            /*!
             * @param task_recorders
             * @return True on success, otherwise False
             */
            virtual bool read(std::vector<boost::shared_ptr<task_recorder::TaskRecorderBase> >& task_recorders)
            {
                return true;
            }

            /*!
             * @return the number of used task recorders
             */
            int getNumberOfTaskRecorders();

            /*!
             * @param request
             * @param response
             * @return True on success, False otherwise
             */
            bool startStreaming(task_recorder::StartStreaming::Request& request,
                                task_recorder::StartStreaming::Response& response);

            /*!
             * @param request
             * @param response
             * @return True on success, False otherwise
             */
            bool stopStreaming(task_recorder::StopStreaming::Request& request,
                               task_recorder::StopStreaming::Response& response);

            /*!
             * @param request
             * @param response
             * @return True on success, False otherwise
             */
            bool startRecording(task_recorder::StartRecording::Request& request,
                                task_recorder::StartRecording::Response& response);

            /*!
             * @param request
             * @param response
             * @return True on success, False otherwise
             */
             bool stopRecording(task_recorder::StopRecording::Request& request,
                                task_recorder::StopRecording::Response& response);

            /*!
             * @param request
             * @param response
             * @return True on success, False otherwise
             */
            bool interruptRecording(task_recorder::InterruptRecording::Request& request,
                                    task_recorder::InterruptRecording::Response& response);

            /*!
             * @param request
             * @param response
             * @return True on success, False otherwise
             */
            bool getInfo(task_recorder::GetInfo::Request& request,
                         task_recorder::GetInfo::Response& response);

            /*!
             * @param request
             * @param response
             * @return True on success, False otherwise
             */
            bool getDataSample(task_recorder::GetDataSample::Request& request,
                               task_recorder::GetDataSample::Response& response);

            /*!
             * @param request
             * @param response
             * @return True on success, False otherwise
             */
            bool addDataSamples(task_recorder::AddDataSamples::Request& request,
                                task_recorder::AddDataSamples::Response& response);

            /*!
             * @param request
             * @param response
             * @return True on success, False otherwise
             */
            bool readDataSamples(task_recorder::ReadDataSamples::Request& request,
                                 task_recorder::ReadDataSamples::Response& response);

        protected:

            /*!
             */
            bool initialized_;

            /*!
             */
            TaskRecorderIO<task_recorder::DataSample> recorder_io_;

        private:
                
            /*!
             */
            static const int DATA_SAMPLE_PUBLISHER_BUFFER_SIZE = 10000;
            
            double sampling_rate_;
            ros::Timer timer_;
            int counter_;

            /*! task recorders
             */
            std::vector<boost::shared_ptr<task_recorder::TaskRecorderBase> > task_recorders_;
            
            /*! data samples
             */
            std::vector<task_recorder::DataSample> data_samples_;
            task_recorder::DataSample last_combined_data_sample_;
            boost::mutex last_combined_data_sample_mutex_;

            /*
             */
            std::vector<task_recorder::StartStreaming::Request> start_streaming_requests_;
            std::vector<task_recorder::StartStreaming::Response> start_streaming_responses_;
            std::vector<task_recorder::StopStreaming::Request> stop_streaming_requests_;
            std::vector<task_recorder::StopStreaming::Response> stop_streaming_responses_;
            std::vector<task_recorder::StartRecording::Request> start_recording_requests_;
            std::vector<task_recorder::StartRecording::Response> start_recording_responses_;
            std::vector<task_recorder::StopRecording::Request> stop_recording_requests_;
            std::vector<task_recorder::StopRecording::Response> stop_recording_responses_;
            std::vector<task_recorder::InterruptRecording::Request> interrupt_recording_requests_;
            std::vector<task_recorder::InterruptRecording::Response> interrupt_recording_responses_;

            /*
             */
            ros::Publisher data_sample_publisher_;
            ros::ServiceServer start_streaming_service_server_;
            ros::ServiceServer stop_streaming_service_server_;
            ros::ServiceServer start_recording_service_server_;
            ros::ServiceServer stop_recording_service_server_;
            ros::ServiceServer interrupt_recording_service_server_;
            ros::ServiceServer get_info_service_server_;
            ros::ServiceServer get_data_sample_service_server_;
            ros::ServiceServer add_data_samples_service_server_;
            ros::ServiceServer read_data_samples_service_server_;

            /*!
             */
            ros::Publisher stop_recording_publisher_;

            /*!
             * @param timer_event 
             */
            void timerCB(const ros::TimerEvent& timer_event);
            
            /*!
             * @param time_stamp
             * @return True if last sample is updated, otherwise False
             */
            bool setLastDataSample(const ros::Time& time_stamp);

    };
}
#endif
