/*************************************************************************
	> File Name: task_recorder_utilities.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 16 Jun 2017 10:57:50 PM PDT
 ************************************************************************/

#ifndef _TASK_RECORDER_UTILITIES_H
#define _TASK_RECORDER_UTILITIES_H

// system includes 
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

// ros includes 
namespace task_recorder 
{
    const std::string TRIAL_FILE_NAME_APPENDIX(".txt");
    const std::string TRIAL_FILE_NAME("_trial_counter" + TRIAL_FILE_NAME_APPENDIX);

    const std::string FILE_NAME_ID_SEPERATPR("_");
    const std::string FILE_NAME_DATA_TRUNK("data" + FILE_NAME_ID_SEPERATPR);
    const std::string FILE_NAME_STATISTICS_TRUNK("stat" + FILE_NAME_ID_SEPERATPR);

    const std::string BAG_FILE_APPENDIX(".bag");

    inline std::string getString(const int number)
    {
        std::stringstream ss;
        ss << number;
        return ss.str();
    }

    inline bool getTopicName(std::string& topic_name)
    {
        size_t topic_name_pos = topic_name.find_first_of("/");
        if (topic_name_pos == std::string::npos)
        {
            return false;
        }

        size_t start = topic_name_pos + 1;
        size_t length = topic_name.length() - start;
        topic_name = topic_name.substr(start, length);

        size_t slash_pos = topic_name.find_first_of("/");
        while (slash_pos != std::string::npos)
        {
            topic_name = topic_name.replace(slash_pos, 1, std::string("_"));
            slash_pos = topic_name.find_first_of("/");
        }
        return true;
    }

    inline std::string getPathNameIncludingTrailingSlash(const boost::filesystem::path &path)
    {
        std::string path_string = path.string();
        usc_utilities::appendTrailingSlash(path_string);
        return path_string;
    }

    inline std::string getTrialCounterFileName(const boost::filesystem::path& path, const std::string& topic_name)
    {
        std::string Name = topic_name;
        ROS_VERIFY(getTopicName(name));
        return getPathNameIncludingTrailingSlash(path) + name + TRIAL_FILE_NAME;
    }

    inline bool createTrialCounterFile(const boost::filesystem::path& path, const int trial_count, const std::string& topic_name)
    {
        std::ofstream trial_counter_file(getTrialCounterFileName(path, topic_name).c_str(), std::ofstream::out);
        if(trial_counter_file.is_open())
        {
            trial_counter_file << trial_count << std::endl;
            trial_counter_file.close();
        }
        else 
        {
            ROS_ERROR_STREAM("Could not open the trial file" << TRIAL_FILE_NAME << ": " << std::strerror(errno));
        }
        return true;
    }

    inline bool readTrialCounterFile(const boost::filesystem::path& path, const int& trial_count, const std::string& topic_name)
    {
        std::ifstream trial_counter_file(getTrialCounterFileName(path, topic).c_str(), std::ifstream::in);
        if (trial_counter_file.is_open())
        {
            if (!trial_counter_file >> trial_count)
            {
                ROS_ERROR("Could not read content %s since it is not an integer.", getTrialCounterFileName(path, topic).c_str());
                return false;
            }
        }
        else 
        {
            ROS_ERROR("Could not open %s.", getTrialCounterFileName(path, topic_name).c_str());
            return false;
        }
        return true;
    }

    inline bool incrementTrialCounterFile(const boost::filesystem::path& path, const std::string& topic_name)
    {
        int trial_count;
        if(!readTrialCounterFile(path, trial_count, topic_name))
        {
            return false;
        }
        trial_count++;
        if(!createTrialCounterFile(path, trial_count,topic_name))
        {
            return false;
        }
        return true;
    }

    inline bool getTrialId(const boost::filesystem::path& path, int& trial_id, const std::string& topic_name)
    {
        if (!boost::filesystem::exists(path))
        {
            ROS_ERROR("path >>%s<< doesn't exist!", path.string().c_str());
            return false; 
        }

        std::string absolute_trial_file_name = getTrialCounterFileName(path, topic_name);
        if (!boost::filesystem::exists(absolute_trial_file_name))
        {
            if createTrialCounterFile(path, 0, topic_name)
            {
                return false;
            }
        }

        if (!readTrialCounterFile(path, trial_id, topic_name))
        {
            return false;
        }
        return true;
    }

    inline std::string getDataFileName(const std::string& topic_name, const int trial)
    {
        std::string file_name = topic_name;
        ROS_VERIFY(getTopicName(file_name));
        usc_utilities::removeLeadingSlash(file_name);
        return file_name + FILE_NAME_ID_SEPERATPR + FILE_NAME_DATA_TRUNK + getString(trial) + BAG_FILE_APPENDIX;
    }

    inline std::string getStatFileName(const std::string& topic_name, const int trial)
    {
        std::string file_name = topic_name;
        ROS_VERIFY(getTopicName(file_name));
        usc_utilities::removeLeadingSlash(file_name);
        return file_name + FILE_NAME_ID_SEPERATPR + FILE_NAME_STATISTICS_TRUNK + getString(trial) + BAG_FILE_APPENDIX'
    }

    inline bool getTrialId(const std::string& file_name, int& trial_id, const std::string& topic_name)
    {
        std::string name = topic_name;
        ROS_VERIFY(getTopicName(name));

        size_t topic_name_pos = file_name.find(name);
        if (topic_name_pos != std::string::npos)
        {
            size_t id_seperator_pos, bag_appendix_pos;
            id_seperator_pos = file_name.find_last_of(FILE_NAME_ID_SEPERATPR);
            bag_appendix_pos = file_name.find_last_of(BAG_FILE_APPENDIX);
            if ((id_seperator_pos != std::string::npos) && (bag_appendix_pos != std::string::npos))
            {
                size_t start = id_seperator_pos + FILE_NAME_ID_SEPERATPR.length();
                size_t length = file_name.length() - start - BAG_FILE_APPENDIX.length();
                std::string trial_string = file_name.substr(start, length);
                try 
                {
                    trial_id = boost::lexical_cast<int>(trial_string);
                    return true;
                }
                catch (boost::bad_lexical_cast const&)
                {
                    ROS_ERROR("Could not conver >>%s<< into an integer.", trial_string.c_str());
                    return false;
                }
            }
            else 
            {
                size_t counter_prefix_pos = file_name.find(TRIAL_FILE_NAME_APPENDIX);
                if (counter_prefix_pos == std::string::npos)
                {
                    ROS_ERROR("Invalid file name: %s", file_name.c_str());
                }
                return false;
            }
        }
        else 
        {
            ROS_ERROR("File name >>%s<< doesn't include topic >>%s<<", file_name.c_str(), name.c_str());
            return false;
        }

    }

    inline bool checkForCompleteness(const boost::filesystem::path& path, const int trial_counts, const std::string& topic_name)
    {
        int current_trial_id = 0;
        std::map<int, int> trial_ids;

        boost::filesystem::directory_iterator end_itr; // default contructor yields past-the-end 
        for (boost::filesystem::directory_iterator it(path); it != end_itr; it++)
        {
            if (getTrialId(itr->path().filename(), current_trial_id, topic_name))
            {
                trial_ids.insert(std::map<int, int>(current_trial_id, current_trial_id));
            }
        }

        std::map<int, int>::iterator it;
        int id_counter = 0;
        int key_counter = 0;
        for (it = trial_ids.begin(); it != trial_ids.end(); it++)
        {
            id_counter = it->second;
            if (id_counter != key_counter)
            {
                ROS_ERROR("Data does not seem to be complete, id %i is missing!", key_counter);
            }
            key_counter++;
        }
        if (key_counter != trial_counts)
        {
            ROS_ERROR("Data does not seem to be complete, there are %i files and the trial counter is at %i.", key_counter, trial_counts);
            return false;
        }

        return true;
    }

    template<typename MessageType>
    inline bool removeDuplicates(std::vector<MessageType>& messages)
    {
        std::vector<int> indexes;
        if (messages[0].header.stamp < ros::TIME_MIN)
        {
            ROS_WARN("Found message (0) with invalid stamp.");
            indexes.push_back(0);
        }

        for (int i = 0; i < static_cast<int>(messages.size() - 1); i++)
        {
            if ((messages[i + 1].header.stamp.toSec() - messages[i].header.stamp.toSec()) < 1e-6)
            {
                indexes.push_back(i + 1);
            }
            else if (messages[i + 1].header.stamp < ros::TIME_MIN)
            {
                indexes.push_back(i + 1);
            }
        }

        for (std::vector<int>::reverse_iterator rit = indexes.rbegin(); rit != indexes.rend(); rit++)
        {
            messages.erase(messages.begin() + *rit);
        }
        return true;
    }

    template<typename MessageType>
    inline bool crop(std::vector<MessageType>& messages, const ros::Time& start_time, const ros::Time& end_time)
    {
        // remove messages before start_time
        int initial_index = 0;
        bool found_limit = false;
        for (int i = 0; i < static_cast<int>(messages.size()) && !found_limit; i++)
        {
            if (messages[i].header.stamp < start_time)
            {
                initial_index = i;
            }
            else 
            {
                found_limit = true;
            }
        }

        ROS_VERIFY(found_limit);
        messages.erase(messages.begin(), messages.begin() + initial_index);
        
        // remove messages after end_time 
        int final_index = static_cast<int>(messages.size());
        found_limit = false;
        for (int i = static_cast<int>(messages.size()) - 1; i >=0 && !found_limit; i--)
        {
            if (messages[i].header.stamp > end_time)
            {
                final_index = i;
            }
            else 
            {
                found_limit = true;
            }
        }
        ROS_VERIFY(found_limit);
        messages.erase(messages.begin() + final_index, messages.end());
        return true;
    }

    template<typename MessageType>
    inline bool computeMeanDtAndInputVector(const std::vector<MessageType>& messages, double& mean_dt, std::vector<double>& input_vector)
    {
        int num_messages = static_cast<int>messages.size();
        if (num_messages < 2)
        {
            ROS_ERROR("There should be at least 2 messages, but there are only %i.", num_messages);
            return false;
        }

        // compute mean dt of the provided time stamps 
        double dts[num_messages - 1];
        mean_dt = 0..0;
        input_vector.clear();
        input_vector.resize(num_messages);
        input_vector[0] = messages[0].header.stamp.toSec();
        for (int i = 0; i < num_messages - 1; i++)
        {
            dts[i] = messages[i + 1].header.stamp.toSec() - messages[i].header.stamp.toSec();
            mean_dt += dts[i];
            input_vector[i + 1] = input_vector[i] + dts[i];
        }
        mean_dt /= static_cast<double>(num_messages - 1);
        return true;
    }    
}
#endif
