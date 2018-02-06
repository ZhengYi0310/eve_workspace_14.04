/*************************************************************************
	> File Name: task_recorder_manager_client.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 27 Jun 2017 10:05:51 AM PDT
 ************************************************************************/

#include <task_recorder/StartStreaming.h>
#include <task_recorder/StopStreaming.h>
#include <task_recorder/StartRecording.h>
#include <task_recorder/StopRecording.h>
#include <task_recorder/InterruptRecording.h>
#include <task_recorder/GetDataSample.h>
#include <task_recorder/GetInfo.h>
#include <task_recorder/AddDataSamples.h>
#include <task_recorder/ReadDataSamples.h>

#include <task_recorder_utilities/task_description_utilities.h>
#include <usc_utilities/services.h>

#include <task_recorder/task_recorder_manager_client.h>

namespace task_recorder
{
    TaskRecorderManagerClient::TaskRecorderManagerClient(bool block_until_all_services_are_available) : node_handle_(ros::NodeHandle()), is_online_(false)
    {
        start_streaming_service_client_ = node_handle_.serviceClient<task_recorder::StartStreaming>(std::string("/TaskRecorderManager/start_streaming"));
        stop_streaming_service_client_ = node_handle_.serviceClient<task_recorder::StopStreaming>(std::string("/TaskRecorderManager/stop_streaming"));
        start_recording_service_client_ = node_handle_.serviceClient<task_recorder::StartRecording>(std::string("/TaskRecorderManager/start_recording"));
        stop_recording_service_client_ = node_handle_.serviceClient<task_recorder::StopRecording>(std::string("/TaskRecorderManager/stop_recording"));
        interrupt_recording_service_client_ = node_handle_.serviceClient<task_recorder::InterruptRecording>(std::string("/TaskRecorderManager/interrupt_recording"));
        get_data_sample_service_client_ = node_handle_.serviceClient<task_recorder::GetDataSample>(std::string("/TaskRecorderManager/get_data_sample"));
        add_data_samples_service_client_ = node_handle_.serviceClient<task_recorder::AddDataSamples>(std::string("/TaskRecorderManager/add_data_samples"));
        read_data_samples_service_client_ = node_handle_.serviceClient<task_recorder::ReadDataSamples>(std::string("/TaskRecorderManager/read_data_samples"));
        get_info_service_client_ = node_handle_.serviceClient<task_recorder::GetInfo>(std::string("/TaskRecorderManager/get_info"));

        if (block_until_all_services_are_available)
        {
            waitForServices();
        }
    }

    void TaskRecorderManagerClient::waitForServices()
    {
        usc_utilities::waitFor(start_streaming_service_client_);
        usc_utilities::waitFor(stop_streaming_service_client_);
        usc_utilities::waitFor(start_recording_service_client_);
        usc_utilities::waitFor(stop_recording_service_client_);
        usc_utilities::waitFor(interrupt_recording_service_client_);
        usc_utilities::waitFor(get_data_sample_service_client_);
        usc_utilities::waitFor(add_data_samples_service_client_);
        usc_utilities::waitFor(read_data_samples_service_client_);
        usc_utilities::waitFor(get_info_service_client_);
        is_online_ = true;
    }

    bool TaskRecorderManagerClient::checkForServices()
    {
        is_online_ = true;
        is_online_ = is_online_ && usc_utilities::isReady(start_streaming_service_client_);
        is_online_ = is_online_ && usc_utilities::isReady(stop_streaming_service_client_);
        is_online_ = is_online_ && usc_utilities::isReady(start_recording_service_client_);
        is_online_ = is_online_ && usc_utilities::isReady(stop_recording_service_client_);
        is_online_ = is_online_ && usc_utilities::isReady(interrupt_recording_service_client_);
        is_online_ = is_online_ && usc_utilities::isReady(get_data_sample_service_client_);
        is_online_ = is_online_ && usc_utilities::isReady(add_data_samples_service_client_);
        is_online_ = is_online_ && usc_utilities::isReady(read_data_samples_service_client_);
        is_online_ = is_online_ && usc_utilities::isReady(get_info_service_client_);
        return is_online_;
    }

    bool TaskRecorderManagerClient::startStreaming()
    {
        if (!servicesAreReady())
        {
            waitForServices();
        }

        task_recorder::StartStreaming::Request start_request;
        task_recorder::StartStreaming::Response start_response;
        if (!start_streaming_service_client_.call(start_request, start_response))
        {
            ROS_ERROR("Problems when calling >%s<.", start_streaming_service_client_.getService().c_str());
            return false;
        }

        if (start_response.return_code != task_recorder::StartStreaming::Response::SERVICE_CALL_SUCCESSFUL)
        {
            ROS_ERROR("Service >%s< was not successful: %s", start_streaming_service_client_.getService().c_str(), start_response.info.c_str());
            return false;
        }
    }

    bool TaskRecorderManagerClient::stopStreaming()
    {
        if (!servicesAreReady())
        {
            waitForServices();
        }
        task_recorder::StopStreaming::Request stop_request;
        task_recorder::StopStreaming::Response stop_response;

        if (!stop_streaming_service_client_.call(stop_request, stop_response))
        {
            ROS_ERROR("Problems when calling >%s<.", stop_streaming_service_client_.getService().c_str());
            return false;
        }

        if(stop_response.return_code != task_recorder::StopStreaming::Response::SERVICE_CALL_SUCCESSFUL)
        {
            ROS_ERROR("Service >%s< was not successful: %s", stop_streaming_service_client_.getService().c_str(), stop_response.info.c_str());
            return false;
        }
  
        ROS_INFO_STREAM_COND(!stop_response.info.empty(), stop_response.info);
        return true;
    }

    bool TaskRecorderManagerClient::startRecording(const std::string description,
                                                   const int id,
                                                   ros::Time& start_time)
    {
        task_recorder::Description task_description;
        task_description.id = id;
        task_description.description.assign(description);
        return startRecording(task_description, start_time);
    }

    bool TaskRecorderManagerClient::startRecording(const task_recorder::Description& description,
                                                   ros::Time& start_time)
    {
        ROS_DEBUG("Start recording description >%s< with id >%i<.", description.description.c_str(), description.id);
        if(!servicesAreReady())
        {
            waitForServices();
        }

        task_recorder::StartRecording::Request start_request;
        start_request.description = description;
        task_recorder::StartRecording::Response start_response;
        if(!start_recording_service_client_.call(start_request, start_response))
        {
            ROS_ERROR("Problems when calling >%s<.", start_recording_service_client_.getService().c_str());
            return false;
        }
  
        if(start_response.return_code != task_recorder::StartRecording::Response::SERVICE_CALL_SUCCESSFUL)
        {
            ROS_ERROR("Service >%s< was not successful: %s", start_recording_service_client_.getService().c_str(), start_response.info.c_str());
            return false;
        }
  
        start_time = start_response.start_time;
        ROS_INFO_STREAM_COND(!start_response.info.empty(), start_response.info);
        return true;
    }

    bool TaskRecorderManagerClient::stopRecording(const ros::Time& start_time,
                                                  const ros::Time& end_time,
                                                  const int num_samples,
                                                  std::vector<task_recorder::DataSample>& messages,
                                                  const bool stop_recording)
    {
        std::vector<std::string> no_message_names;
        return stopRecording(start_time, end_time, num_samples, no_message_names, messages, stop_recording);
    }

    bool TaskRecorderManagerClient::stopRecording(const ros::Time& start_time,
                                                  const ros::Time& end_time,
                                                  const int num_samples,
                                                  const std::vector<std::string>& message_names,
                                                  std::vector<task_recorder::DataSample>& messages,
                                                  const bool stop_recording)
    {
        if(!servicesAreReady())
        {
            waitForServices();
        }
  
        task_recorder::StopRecording::Request stop_request;
        task_recorder::StopRecording::Response stop_response;
        stop_request.crop_start_time = start_time;
        stop_request.crop_end_time = end_time;
        stop_request.num_samples = num_samples;
        stop_request.message_names = message_names;
        stop_request.stop_recording = stop_recording;
  
        if(!stop_recording_service_client_.call(stop_request, stop_response))
        {
            ROS_ERROR("Problems when calling >%s<.", stop_recording_service_client_.getService().c_str());
            return false;
        }
  
        if(stop_response.return_code != task_recorder::StopRecording::Response::SERVICE_CALL_SUCCESSFUL)
        {
            ROS_ERROR("Service >%s< was not successful: %s (%i)", stop_recording_service_client_.getService().c_str(), stop_response.info.c_str(), stop_response.return_code);
            return false;
        }
  
        messages = stop_response.filtered_and_cropped_messages;
        ROS_INFO_STREAM_COND(!stop_response.info.empty(), stop_response.info);
        return true;
    }

    bool TaskRecorderManagerClient::stopRecording(const ros::Time& start_time,
                                                  const ros::Time& end_time,
                                                  const std::vector<std::string>& message_names,
                                                  std::vector<task_recorder::DataSample>& messages,
                                                  const bool stop_recording)
    {
        bool is_recording;
        ros::Time first;
        ros::Time last;
        double sampling_rate;
        if (!getInfo(is_recording, first, last, sampling_rate))
        {
            ROS_ERROR("Could not stop recording.");
            return false;
        }

        if (!is_recording)
        {
            ROS_ERROR("Task recorders are not recording, cannot stop.");
            return false;
        }

        if (start_time < first)
        {
            ROS_ERROR("Requested start time >%f< is invalid. First recorded sample has time stamp is >%f<.", start_time.toSec(), first.toSec());
            return false;
        }

        if(end_time > last)
        {
            ROS_ERROR("Requested end time >%f< is invalid. Last recorded sample has time stamp is >%f<.", end_time.toSec(), last.toSec());
            return false;
        }

        ros::Duration duration = end_time - start_time;
        const int num_samples = static_cast<int>(duration.toSec() * sampling_rate);
        ROS_INFO("Recorded >%.2f< seconds and asking for >%i< samples.", duration.toSec(),  num_samples);
        return stopRecording(start_time, end_time, num_samples, message_names, messages, stop_recording);
    }

    bool TaskRecorderManagerClient::stopRecording(const ros::Time& start_time,
                                                  const std::vector<std::string>& message_names,
                                                  std::vector<task_recorder::DataSample>& messages,
                                                  const bool stop_recording)
    {
        bool is_recording;
        ros::Time first;
        ros::Time last;
        double sampling_rate;
        if(!getInfo(is_recording, first, last, sampling_rate))
        {
            ROS_ERROR("Could not stop recording.");
            return false;
        }
  
        if(!is_recording)
        {
            ROS_ERROR("Task recorders are not recording, cannot stop.");
            return false;
        }
  
        if(start_time < first)
        {
            ROS_ERROR("Requested start time >%f< is invalid. First recorded sample has time stamp is >%f<.", start_time.toSec(), first.toSec());
            return false;
        }
  
        ros::Duration duration = last - start_time;
        const int num_samples = static_cast<int>(duration.toSec() * sampling_rate);
        ROS_INFO("Recorded >%f< seconds and asking for >%i< samples.", duration.toSec(), num_samples);
        return stopRecording(start_time, last, num_samples, message_names, messages, stop_recording);
    }

    bool TaskRecorderManagerClient::stopRecording(const std::vector<std::string>& message_names,
                                                  std::vector<task_recorder::DataSample>& messages,
                                                  const bool stop_recording)
    {
        bool is_recording;
        ros::Time first;
        ros::Time last;
        double sampling_rate;
  
        if(!getInfo(is_recording, first, last, sampling_rate))
        {
            ROS_ERROR("Could not stop recording.");
            return false;
        }
  
        if(!is_recording)
        {
            ROS_ERROR("Task recorders are not recording, cannot stop.");
            return false;
        }
        ros::Duration duration = last - first;
        const int num_samples = static_cast<int>(duration.toSec() * sampling_rate);
        ROS_INFO("Recorded >%f< seconds and asking for >%i< samples.", duration.toSec(), num_samples);
        return stopRecording(first, last, num_samples, message_names, messages, stop_recording);
    }

    bool TaskRecorderManagerClient::interruptRecording()
    {
        if(!servicesAreReady())
        {
            waitForServices();
        }
        task_recorder::InterruptRecording::Request interrupt_request;
        task_recorder::InterruptRecording::Response interrupt_response;
    
        if(!interrupt_recording_service_client_.call(interrupt_request, interrupt_response))
        {
            ROS_ERROR("Problems when calling >%s<.", interrupt_recording_service_client_.getService().c_str());
            return false;
        }
  
        if(interrupt_response.return_code != task_recorder::InterruptRecording::Response::SERVICE_CALL_SUCCESSFUL)
        {
            ROS_ERROR("Service >%s< was not successful: %s (%i)", interrupt_recording_service_client_.getService().c_str(), interrupt_response.info.c_str(), interrupt_response.return_code);
            return false;
        }
  
        ROS_INFO_STREAM_COND(!interrupt_response.info.empty(), interrupt_response.info);
        return true;
    }

    bool TaskRecorderManagerClient::getDataSample(const task_recorder::Description& description,
                                                  task_recorder::DataSample& data_sample)
    {
        if(!servicesAreReady())
        {
            waitForServices();
        }
        task_recorder::GetDataSample::Request get_data_sample_request;
        get_data_sample_request.description = description;
        task_recorder::GetDataSample::Response get_data_sample_response;
        if(!get_data_sample_service_client_.call(get_data_sample_request, get_data_sample_response))
        {
            ROS_ERROR("Problems when calling >%s<.", get_data_sample_service_client_.getService().c_str());
            return false;
        }
  
        
        if(get_data_sample_response.return_code != task_recorder::GetDataSample::Response::SERVICE_CALL_SUCCESSFUL)
        {
            ROS_ERROR("Service >%s< was not successful: %s (%i)", get_data_sample_service_client_.getService().c_str(), get_data_sample_response.info.c_str(), get_data_sample_response.return_code);
            return false;
        }
  
        data_sample = get_data_sample_response.data_sample;
        ROS_INFO_STREAM_COND(!get_data_sample_response.info.empty(), get_data_sample_response.info);
        return true;
    }

    bool TaskRecorderManagerClient::addDataSamples(const task_recorder::Description& description,
                                               const std::vector<task_recorder::DataSample>& data_samples)
    {
        if(!servicesAreReady())
        {
            waitForServices();
        }
  
        task_recorder::AddDataSamples::Request add_data_samples_request;
        add_data_samples_request.description = description;
        add_data_samples_request.data_samples = data_samples;
        task_recorder::AddDataSamples::Response add_data_samples_response;
  
        if(!add_data_samples_service_client_.call(add_data_samples_request, add_data_samples_response))
        {
            ROS_ERROR("Problems when calling >%s<.", add_data_samples_service_client_.getService().c_str());
            return false;
        }
  
        if(add_data_samples_response.return_code != task_recorder::AddDataSamples::Response::SERVICE_CALL_SUCCESSFUL)
        {
            ROS_ERROR("Service >%s< was not successful: %s (%i)", add_data_samples_service_client_.getService().c_str(), add_data_samples_response.info.c_str(), add_data_samples_response.return_code);
            return false;
        }
  
        ROS_INFO_STREAM_COND(!add_data_samples_response.info.empty(), add_data_samples_response.info);
        return true;
    }

    bool TaskRecorderManagerClient::readDataSamples(const task_recorder::Description& description,
                                                    std::vector<task_recorder::DataSample>& data_samples)
    {
        if(!servicesAreReady())
        {
            waitForServices();
        }
  
        task_recorder::ReadDataSamples::Request read_data_samples_request;
        read_data_samples_request.description = description;
        task_recorder::ReadDataSamples::Response read_data_samples_response;
        if(!read_data_samples_service_client_.call(read_data_samples_request, read_data_samples_response))
        {
            ROS_ERROR("Problems when calling >%s<.", read_data_samples_service_client_.getService().c_str());
            return false;
        }
  
        if(read_data_samples_response.return_code != task_recorder::ReadDataSamples::Response::SERVICE_CALL_SUCCESSFUL)
        {
            ROS_ERROR("Service >%s< was not successful: %s (%i)", read_data_samples_service_client_.getService().c_str(), read_data_samples_response.info.c_str(), read_data_samples_response.return_code);
            return false;
        }
  
        data_samples = read_data_samples_response.data_samples;
        ROS_INFO_STREAM_COND(!read_data_samples_response.info.empty(), read_data_samples_response.info);
        return true;
    }

    bool TaskRecorderManagerClient::getInfo(const task_recorder::Description& description,
                                            std::string& abs_file_name)
    {
        if(!servicesAreReady())
        {
            waitForServices();
        }
  
        task_recorder::GetInfo::Request get_info_request;
        get_info_request.description = description;
        task_recorder::GetInfo::Response get_info_response;
  
        if(!get_info_service_client_.call(get_info_request, get_info_response))
        {
            ROS_ERROR("Problems when calling >%s<.", get_info_service_client_.getService().c_str());
            return false;
        }
  
        if(get_info_response.return_code != task_recorder::GetInfo::Response::SERVICE_CALL_SUCCESSFUL)
        {
            ROS_ERROR("Service >%s< was not successful: %s (%i)", get_info_service_client_.getService().c_str(), get_info_response.info.c_str(), get_info_response.return_code);
            return false;
        }
  
        abs_file_name.assign(get_info_response.file_name);
        return true;
    }

    bool TaskRecorderManagerClient::getInfo(bool &is_recording, ros::Time& first, ros::Time& last, double& sampling_rate)
    {
        if(!servicesAreReady())
        {
            waitForServices();
        }
  
        task_recorder::GetInfo::Request get_info_request;
        task_recorder::GetInfo::Response get_info_response;
  
        if(!get_info_service_client_.call(get_info_request, get_info_response))
        {
            ROS_ERROR("Problems when calling >%s<.", get_info_service_client_.getService().c_str());
            return false;
        }
  
        if(get_info_response.return_code != task_recorder::GetInfo::Response::SERVICE_CALL_SUCCESSFUL)
        {
            ROS_ERROR("Service >%s< was not successful: %s (%i)", get_info_service_client_.getService().c_str(), get_info_response.info.c_str(), get_info_response.return_code);
            return false;
        }
  
        ROS_INFO_STREAM_COND(!get_info_response.info.empty(), get_info_response.info);
        is_recording = get_info_response.is_recording;
        first = get_info_response.first_recorded_time_stamp;
        last = get_info_response.last_recorded_time_stamp;
        sampling_rate = get_info_response.sampling_rate;
        return true;
    }
    
}

