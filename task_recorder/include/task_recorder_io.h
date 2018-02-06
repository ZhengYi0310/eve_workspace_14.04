/*************************************************************************
	> File Name: task_recorder_io.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 19 Jun 2017 12:38:53 PM PDT
 ************************************************************************/

#ifndef _TASK_RECORDER_IO_H
#define _TASK_RECORDER_IO_H

// ros includes 
#include <ros/ros.h>
#include <ros/package.h>

#include <rosbag/bag.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/file_io.h>

#include <boost/filesystem.hpp>

// local includes 
#include <task_recorder/task_recorder_utilities.h>
#include <task_recorder/AccumulatedTrialStatistics.h>

namespace task_recorder
{
    template<class MessageType>
    class TaskRecorderIO
    {
        public:
            
            typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;

            TaskRecorderIO() {};
            virtual ~TaskRecorderIO() {};

            /*!
             * @param node_handle
             * @param topic_name
             * @return 
             */
            bool initialize(ros::NodeHandle& node_handle,
                            const std::string& topic_name);

            /*!
             * @param id
             */
            void setId(const int id);

            /*!
             * @return 
             */
            bool writeRecordedData();
            bool writeRawData();

            /*!
             * @return 
             */
            bool writeStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& accumulated_trial_stats_vec);

            /*!
             */
            ros::NodeHandle node_handle_;
            std::string topic_name_;

            /*!
             */
            std::vector<MessageType> messsages_;

        private:
            
            int id_;
            int trial_;

            std::string data_directory_name_;
            boost::filesystem::path absolute_path_directory_path_;

            bool checkForDirectory();
    };

    template<class MessageType>
    TaskRecorderIO<MessageType>::initialize(ros::NodeHandle& node_handle,
                                            const std::string& topic_name)
    {
        node_handle_ = node_handle;
        topic_name_ = topic_name;

        ROS_INFO("Initializing the task recorder for the topic named %s.", topic_name.c_str());
        std::string package_name;
        ROS_VERIFY(usc_utilities::read(node_handle_, "package_name", package_name));
        std::string package_path = ros::package::getPath(package_name);
        usc_utilities::appendTrailingSlash(package_path);

        ROS_VERIFY(usc_utilities::read(node_handle_, "data_directory_name_", data_directory_name_));
        usc_utilities::appendTrailingSlash(data_directory_name_);

        data_directory_name_.assign(package_path + data_directory_name_);

        try
        {
            boost::filesystem::create_directories(data_directory_name_);
        }
        catch (std::exception e)
        {
            ROS_ERROR_STREAM("Exception: " << e.what() << std::endl << "Data directory " << data_directory_name_ << " could not be created!");
            return false;
        }

        return true;
    }
    
    template<class MessageType>
    void TaskRecorderIO<MessageType>::setId(const int id)
    {
        id_ = id;

        // check whether directory exists, if not, create it 
        absolute_path_directory_path_ = boost::filesystem::path(data_directory_name_ + FILE_NAME_DATA_TRUNK + getString(id_));
        checkForDirectory();

        ROS_VERIFY(getTrialId(absolute_path_directory_path_, trial_, topic_name_));
        ROS_VERIFY(checkForCompleteness(absolute_path_directory_path_, trial_, topic_name_));
    }

    template<class MessageType>
    TaskRecorderIO<MessageType>::checkForDirectory()
    {
        // check for the directory, if not, create it 
        if (boost::filesystem::exists(absolute_path_directory_path_))
        {
            return true;
        }

        else 
        {
            if (!boost::filesystem::create_directory(absolute_path_directory_path_))
            {
                ROS_ERROR_STREAM("Could not create directory " << absolute_path_directory_path_.filename() << " :" << std::strerror(errno));
                return false;
            }
        }
        return true;
    }

    template<class MessageType>
    bool TaskRecorderIO<MessageType>::writeRecordedData()
    {
        std::string file_name = getPathNameIncludingTrailingSlash(absolute_path_directory_path_) + getDataFileName(topic_name_, trial_);

        ROS_INFO("Writting data to >%s<.", file_name.c_str());
        try 
        {
            rosbag::Bag bag;
            bag.open(file_name, rosbag::bagmode::Write);
            for (int i = 0; i < static_cast<int>(messsages_.size()); i++)
            {
                bag.write(file_name, messsages_[i].header.stamp, messsages_[i]);
            }
            bag.close();
        }
        catch (rosbag::BagIOException ex)
        {
            ROS_ERROR("Problem when writing to bag file named %s!", file_name.c_str());
            return false;
        }
        
        if (!incrementTrialCounterFile(absolute_path_directory_path_, topic_name_))
        {
            ROS_ERROR("Can't increase the trial counter!");
            return false;
        }

        ROS_VERIFY(getTrialId(absolute_path_directory_path_, trial_, topic_name_));
        ROS_VERIFY(checkForCompleteness(absolute_path_directory_path_, trial_, topic_name_));
        return true;
    }

    template<class MessageType>
    bool TaskRecorderIO<MessageType>::writeRawData()
    {
        std::string directory_name = getPathNameIncludingTrailingSlash(absolute_path_directory_path_) + std::string("raw");
        boost::filesystem::path directory_path = boost::filesystem::path(directory_name);
        if (!boost::filesystem::exists(directory_path))
        {
            if (!boost::filesystem::create_directory(directory_path))
            {
                ROS_ERROR_STREAM("Could not create directory " << directory_path.filename() << " :" << std::strerror(errno));
                return false;
            }
        }

        std::string file_name = directory_name + std::string("/") + getDataFileName(topic_name_, trial_);

        return usc_utilities::FileIO<MessageType>::writeToBagFileWithTimeStamps(messsages_, topic_name_, file_name);

        return true;
    }

    template<class MessageType>
    bool TaskRecorderIO<MessageType>::writeStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& accumulated_trial_stats_vec)
    {
        std::string file_name = getPathNameIncludingTrailingSlash(absolute_path_directory_path_) + getStatFileName(topic_name_, trial_);

        try 
        {
            rosbag::Bag bag;
            bag.open(file_name, rosbag:;bagmode::Write);
            for (int i = 0; i < static<int>(accumulated_trial_stats_vec.size()); i++)
            {
                std::vector<task_recorder::AccumulatedTrialStatistics> accumulated_trial_statistics = accumulated_trial_stats_vec[i];
                for (int j = 0; j < static_cast<int>(accumulated_trial_statistics.size()); i++)
                {
                    accumulated_trial_statistics[j].id = id_;
                    bag.write(topic_name_, messsages_[j].header.stamp, accumulated_trial_statistics[j]);
                }
            }
            bag.close();
        }
        catch (rosbag::BagIOException ex)
        {
            ROS_ERROR("Problem when wrtting to bag file named %s : %s", file_name.c_str(), ex.what());
            return false;
        }

        return true;
    }
}
#endif
