/*************************************************************************
	> File Name: task_recorder_base.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Sun 25 Jun 2017 03:25:06 PM PDT
 ************************************************************************/

#ifndef _TASK_RECORDER_BASE_H
#define _TASK_RECORDER_BASE_H

// system includes 
#include <ros/ros.h>
#include <task_recorder/DataSample.h>
#include <task_recorder/TaskRecorderSpecification.h>

#include <string>
#include <boost/shared_ptr.hpp>

// local includes 
#include <task_recorder/StartStreaming.h>
#include <task_recorder/StopStreaming.h>
#include <task_recorder/StartRecording.h>
#include <task_recorder/StopRecording.h>
#include <task_recorder/InterruptRecording.h>

namespace task_recorder
{
    class TaskRecorderBase
    {
        public:

            TaskRecorderBase() {};
            virtual ~TaskRecorderBase() {};
            
            /*!
             * @param task_recorder_specification
             * @return True on success, otherwise False
             */
            virtual bool initialize(const task_recorder::TaskRecorderSpecification& task_recorder_specification) = 0;
            
            /*!
             * @param node_handle The node_handle that is specific to the (particular) task recorder
             * Derived classes can implement this function to retrieve (arm) specific parameters
             * @return True on success, otherwise False
             */
            virtual bool readParams(ros::NodeHandle& node_handle) = 0;

            /*!
             * @param node_handle
             * @param class_name_prefix
             * @return True on success, otherwise False
             */
            virtual bool readParams(ros::NodeHandle& node_handle, const std::string& class_name_prefix) = 0;

            /*!
             * @param request
             * @param response
             * @return True on success, otherwise False
             */
            virtual bool startStreaming(task_recorder::StartStreaming::Request& request,
                                        task_recorder::StartStreaming::Response& response) = 0;

            /*!
             * @param request
             * @param response
             * @return True on success, otherwise False
             */
            virtual bool stopStreaming(task_recorder::StopStreaming::Request& request, 
                                       task_recorder::StopStreaming::Response& response) = 0;
            
            /*!
             * @param request
             * @param response
             * @return True on success, otherwise False
             */
            virtual bool startRecording(task_recorder::StartRecording::Request& request,
                                        task_recorder::StartRecording::Response& response) = 0;

            /*!
             * @param request
             * @param response
             * @return True on success, otherwise False
             */
            virtual bool stopRecording(task_recorder::StopRecording::Request& request,
                                       task_recorder::StopRecording::Response& response) = 0;

            /*!
             * @param request
             * @param response
             * @return True on success, otherwise False
             */
            virtual bool interruptRecording(task_recorder::InterruptRecording::Request& request,
                                            task_recorder::InterruptRecording::Response& response) = 0;

            /*!
             * @param first
             * @param last
             * @return True if successful (i.e. recorder is currently recording), otherwise False
             */
            virtual bool getTimeStamps(ros::Time& first,
                                       ros::Time& last) = 0;

            /*!
             * @param time
             * @param data_sample
             * @return True on success, otherwise False
             */
            virtual bool getSampleData(const ros::Time& time,
                                       task_recorder::DataSample& data_sample) = 0;

            /*!
             * @return All the variable names that the task recorder can record 
             */
            virtual std::vector<std::string> getNames() const = 0;

            /*!
             * @return
             */
            static std::string getClassName();

            /*!
             * This function will be called right before each recording is started
             * @return True on success, otherwise False
             */
            virtual bool startRecording() = 0;

        private:
    };

    typedef boost::shared_ptr<TaskRecorderBase> TaskRecorderBasePtr;
}
#endif
