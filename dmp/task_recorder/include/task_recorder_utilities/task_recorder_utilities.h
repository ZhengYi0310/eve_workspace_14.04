/*************************************************************************
	> File Name: task_recorder_utilities.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 23 Jun 2017 02:49:04 PM PDT
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

//#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

// ros inlcudes 
#include <ros/package.h>

// local includes 
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <task_recorder/Description.h>

namespace task_recorder_utilities 
{
    const std::string TRIAL_FILE_NAME_APPENDIX(".txt");
    const std::string TRIAL_FILE_NAME("_trial_counter" + TRIAL_FILE_NAME_APPENDIX);
    const std::string IGNORE_DAT_FILE_NAME_APPENDIX(".dat");
    const std::string FILE_NAME_ID_SEPERATOR("_");
    const std::string FILE_NAME_TRIAL_SEPARATOR("trial" + FILE_NAME_ID_SEPERATOR);
    const std::string FILE_NAME_STATISTICS_TRUNK_NO_SEPARATOR("stat");
    const std::string FILE_NAME_STATISTICS_TRUNK_SEPERATOR(FILE_NAME_STATISTICS_TRUNK_NO_SEPARATOR + FILE_NAME_ID_SEPERATOR);

    const std::string CLMC_FILE_NAME("d");
    const int MIN_TRIAL_ID = 0;
    const int MAX_TRIAL_ID = 99999;

    const std::string BAG_FILE_APPENDIX(".bag");

    inline std::string getTrialCounterFileName(const boost::filesystem::path& path,
                                               const std::string& topic_name);

    inline void removeBagFileAppendix(std::string& file_name);
    inline void appendBagFileAppendix(std::string& file_name);

    inline bool setCLMCFileName(std::string& file_name, const int trial)
    {
        // error checking 
        if (trial < MIN_TRIAL_ID)
        {
            ROS_ERROR("Trial id >%i< is invalid. It needs to be greater than >%i<.", trial, MIN_TRIAL_ID);
            return false;
        }
        else if (trial > MAX_TRIAL_ID)
        {
            ROS_ERROR("Trial id >%i< is invalid. It needs to be less then >%i<.", trial, MAX_TRIAL_ID);
            return false;
        }

        file_name.assign(CLMC_FILE_NAME);
        std::stringstream ss;
        ss << MIN_TRIAL_ID + 1 + trial;
        file_name.append(ss.str().substr(1, ss.str().length() - 1));
        return true;
    }

    inline std::string getDirectoryPath(const std::string package_name = "task_recorder",
                                        const std::string directory_name = "recorded_data")
    {
        std::string directory_path = ros::package::getPath(package_name);
        usc_utilities::appendTrailingSlash(directory_path);
        directory_path.append(directory_name);
        usc_utilities::appendTrailingSlash(directory_path);
        return directory_path;
    }

    inline bool checkForDirectory(const boost::filesystem::path& absolute_data_directory_path,
                                  bool create_if_nonexistent = true)
    {
        // check for directory, if it does not exist -> create it 
        if (boost::filesystem::exists(absolute_data_directory_path))
        {
            return true;
        }

        else 
        {
            if (create_if_nonexistent)
            {
                if (!boost::filesystem::create_directories(absolute_data_directory_path))
                {
                    ROS_WARN_STREAM("Could not create directory >" << absolute_data_directory_path.filename() << "< : " << std::strerror(errno));
                    return false;
                }

                // create .last_data symlink
                std::string from_file = absolute_data_directory_path.string() + std::string("/.last_data");
                std::string to_file = getTrialCounterFileName(absolute_data_directory_path, "/TaskRecorderManager/data_samples");
                boost::filesystem::path from_path(from_file);
                boost::filesystem::path to_path(to_file);
                boost::filesystem::create_symlink(to_path, from_path);
            }

            else
            {
                ROS_ERROR("Directory >%s< does not exist.", absolute_data_directory_path.string().c_str());
                return false;
            }
        }
        return true;
    }

    inline bool checkForDirectory(const std::string& directory_name, bool create_if_nonexistent = true)
    {
        return checkForDirectory(boost::filesystem::path(directory_name), create_if_nonexistent);
    }

    inline bool checkAndCreateDirectories(const std::string& directory_name, bool create_if_nonexistent = true)
    {
        try
        {
            boost::filesystem::create_directories(directory_name);
        }
        catch (std::exception& p)
        {
            ROS_ERROR("Data directory >%s< could not be created: %s", directory_name.c_str(), p.what());
            return false;
        }
        return true;
    }

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
            ROS_ERROR("topic name doesn't exist!");
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

    inline std::string getPathNameIncludingTrailingSlash(const boost::filesystem::path& path)
    {
        std::string path_string = path.string();
        usc_utilities::appendTrailingSlash(path_string);
        return path_string;
    }

    inline std::string getTrialCounterFileName(const boost::filesystem::path& path,
                                               const std::string& topic_name)
    {
        std::string name = topic_name;
        ROS_VERIFY_MSG(getTopicName(name), "Could not parse topic name >%s<. This should never happen.", name.c_str());
        return getPathNameIncludingTrailingSlash(path) + name + TRIAL_FILE_NAME;
    }


    inline bool createTrialCounterFile(const boost::filesystem::path& path,
                                       const int trial_count,
                                       const std::string& topic_name)
    {
        std::ofstream trial_counter_file(getTrialCounterFileName(path, topic_name).c_str(), std::ios::out);
        if (trial_counter_file.is_open())
        {
            trial_counter_file << trial_count << std::endl;
            trial_counter_file.close();
        }
        else
        {
            ROS_ERROR_STREAM("Could not open trial counter file " << TRIAL_FILE_NAME << " : " << std::strerror(errno));
        }
        return true;
    }

    inline bool readTrialCounterFile(const boost::filesystem::path& path,
                                     int& trial_counter,
                                     const std::string& topic_name)
    {
        std::string absolute_trial_file_name = getTrialCounterFileName(path, topic_name);
        if (!boost::filesystem::exists(absolute_trial_file_name))
        {
            if (!createTrialCounterFile(path, 0, topic_name))
            {
                return false;
            }
        }

        std::ifstream trial_counter_file(getTrialCounterFileName(path, topic_name).c_str());
        if (trial_counter_file.is_open())
        {
            if (!(trial_counter_file >> trial_counter))
            {
                ROS_ERROR("Could not read content of >%s< since it is not an integer.", getTrialCounterFileName(path, topic_name).c_str());
                return false;
            }
        }
        else
        {
            ROS_ERROR("Could not open >%s<.", getTrialCounterFileName(path, topic_name).c_str());
            return false;
        }
        return true;
    }

    inline bool incrementTrialCounterFile(const boost::filesystem::path& path,
                                      const std::string& topic_name)
    {

        int trial_count;
        if (!readTrialCounterFile(path, trial_count, topic_name))
        {
            return false;
        }
        trial_count++;
        if (!createTrialCounterFile(path, trial_count, topic_name))
        {
            return false;
        }
        return true;
    }

    inline bool getTrialId(const boost::filesystem::path& path,
                           int& trial_id,
                           const std::string& topic_name)
    {
        if (!boost::filesystem::exists(path))
        {
            return false;
        }

        std::string absolute_trial_file_name = getTrialCounterFileName(path, topic_name);
        if (!boost::filesystem::exists(absolute_trial_file_name))
        {
            if (!createTrialCounterFile(path, 0, topic_name))
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

    inline std::string getDataFileName(const std::string& topic_name,
                                       const int trial)
    {
        std::string file_name = topic_name;
        ROS_VERIFY(getTopicName(file_name));
        usc_utilities::removeLeadingSlash(file_name);
        // return file_name + FILE_NAME_ID_SEPERATOR + FILE_NAME_DATA_TRUNK2 + getString(trial) + BAG_FILE_APPENDIX;
        return file_name + FILE_NAME_ID_SEPERATOR + FILE_NAME_TRIAL_SEPARATOR + getString(trial) + BAG_FILE_APPENDIX;
    }    

    inline std::string getStatFileName(const std::string& topic_name)
    {
        std::string file_name = topic_name;
        ROS_VERIFY(getTopicName(file_name));
        usc_utilities::removeLeadingSlash(file_name);
        return file_name + FILE_NAME_ID_SEPERATOR + FILE_NAME_STATISTICS_TRUNK_NO_SEPARATOR + BAG_FILE_APPENDIX;
    }

    inline std::string getStatFileName(const std::string& topic_name,
                                       const int trial)
    {
        std::string file_name = topic_name;
        ROS_VERIFY(getTopicName(file_name));
        usc_utilities::removeLeadingSlash(file_name);
        return file_name + FILE_NAME_ID_SEPERATOR + FILE_NAME_STATISTICS_TRUNK_SEPERATOR + getString(trial) + BAG_FILE_APPENDIX;
    }

    inline bool parseDescriptionString(const std::string& description_string, task_recorder::Description& description)
    {
        std::string ds = description_string;
        std::string ds_without_trial = description_string;
        removeBagFileAppendix(ds);

        // set default values 
        description.id = 0;
        description.trial = -1;
        description.description = "";

        // read trial 
        size_t seperator_pos;
        seperator_pos = ds.rfind(FILE_NAME_ID_SEPERATOR + FILE_NAME_TRIAL_SEPARATOR);
        if (seperator_pos != std::string::npos)
        {
            size_t start = seperator_pos + (FILE_NAME_ID_SEPERATOR + FILE_NAME_TRIAL_SEPARATOR).length();
            size_t length = ds.length() - start;
            ds_without_trial = ds.substr(0, seperator_pos + 1);
            std::string trial_string = ds.substr(start, length);

            try
            {
                description.trial = boost::lexical_cast<int>(trial_string);
            }
            catch (boost::bad_lexical_cast const&)
            {
                ROS_ERROR("Could not convert trial >%s< into an integer.", trial_string.c_str());
                return false;
            }
        }

        // read id 
        seperator_pos = ds_without_trial.rfind(FILE_NAME_ID_SEPERATOR);
        if (seperator_pos != std::string::npos)
        {
            size_t sp = seperator_pos + FILE_NAME_ID_SEPERATOR.length();
            size_t length = ds_without_trial.length() - sp;
            description.description = ds_without_trial.substr(0, seperator_pos);
            std::string id_string = ds_without_trial.substr(sp, length);
            try
            {
                description.id = boost::lexical_cast<int>(id_string);
            }
            catch (boost::bad_lexical_cast const&)
            {
                ROS_ERROR("Could not convert id >%s< into an integer.", id_string.c_str());
                return false;
            }
        }
        else
        {
            ROS_ERROR("Invalid description string >%s<. It does not contain a separator.", description_string.c_str());
            return false;
        }

        return true;
    }

    inline bool parseDescriptionString(const std::string& description_string, std::string& description, int& id)
    {
        task_recorder::Description d; 
        if (!parseDescriptionString(description_string, d))
        {
            ROS_ERROR("Can't parse the description string!");
            return false;
        }

        description = d.description;
        id = d.id;
        return true;
    }

    inline void removeBagFileAppendix(std::string& file_name)
    {
        if(file_name.length() >= BAG_FILE_APPENDIX.length())
        {
            std::string appendix = file_name.substr(file_name.length()-BAG_FILE_APPENDIX.length(), file_name.length()-1);
            if(appendix.compare(BAG_FILE_APPENDIX) == 0)
            {
                file_name = file_name.substr(0, file_name.length() - BAG_FILE_APPENDIX.length());
            }
        }
    }

    inline void appendBagFileAppendix(std::string& file_name)
    {
        removeBagFileAppendix(file_name);
        file_name.append(BAG_FILE_APPENDIX);
    }

    

    inline bool getDirectoryList(const boost::filesystem::path& path,
                                 std::vector<std::string>& filenames)
    {
        filenames.clear();
        boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
        for (boost::filesystem::directory_iterator itr(path); itr != end_itr; ++itr)
        {
            size_t ignore_prefix_pos = (itr->path().filename().string()).find(IGNORE_DAT_FILE_NAME_APPENDIX);
            if (ignore_prefix_pos == std::string::npos)
            {
                filenames.push_back(itr->path().filename().string());
            }
    
            else
            {
                ROS_DEBUG("Ignoring file named >%s<.", itr->path().filename().string().c_str());
            }
        }
        return true;
    }

    inline bool getTrialId(const std::string& file_name,
                           int& trial_id,
                           const std::string& topic_name)
    {
        std::string name = topic_name;
        ROS_VERIFY(getTopicName(name));
        // usc_utilities::removeLeadingSlash(name);

        size_t topic_name_pos = file_name.find(name);
        if (topic_name_pos != std::string::npos)
        {
            size_t seperator_pos, bag_prefix_pos;
            // seperator_pos = file_name.find_last_of(FILE_NAME_ID_SEPERATOR);
            seperator_pos = file_name.rfind(FILE_NAME_ID_SEPERATOR);
            bag_prefix_pos = file_name.rfind(BAG_FILE_APPENDIX);
            if ((seperator_pos != std::string::npos) && (bag_prefix_pos != std::string::npos))
            {
                size_t start = seperator_pos + FILE_NAME_ID_SEPERATOR.length();
                size_t length = file_name.length() - start - BAG_FILE_APPENDIX.length();
                std::string trial_string = file_name.substr(start, length);
                try
                {
                    trial_id = boost::lexical_cast<int>(trial_string);
                    return true;
                }
                catch (boost::bad_lexical_cast const&)
                {
                    // TODO: Check whether this is ok...
                    ROS_ERROR("Could not convert >>%s<< into an integer.", trial_string.c_str());
                    return false;
                }
            }
            else
            {
                size_t ignore_prefix_pos = file_name.find(IGNORE_DAT_FILE_NAME_APPENDIX);
                if (ignore_prefix_pos != std::string::npos)
                {
                    ROS_DEBUG("Ignoring file named >%s<.", file_name.c_str());
                    return false;
                }
                size_t counter_prefix_pos = file_name.find(TRIAL_FILE_NAME_APPENDIX);
                if (counter_prefix_pos == std::string::npos)
                {
                    ROS_ERROR("Invalid file name: %s.", file_name.c_str());
                }
                return false;
            }
        }
        return false;
    }

    inline bool checkForCompleteness(const boost::filesystem::path& path,
                                     const int trial_counts,
                                     const std::string& topic_name)
    {
        int current_trial_id = 0;
        std::map<int, int> trial_ids;

        boost::filesystem::directory_iterator end_itr; //default contruction yeilds past-the-end 
        for (boost::filesystem::directory_iterator itr(path); itr != end_itr; itr++)
        {
            if (getTrialId(itr->path().filename().string(), current_trial_id, topic_name))
            {
                trial_ids.insert(std::pair<int, int>(current_trial_id, current_trial_id));
            }
        }

        std::map<int, int>::iterator it;
        int id_counter = 0;
        int key_counter = 0;
        for (it = trial_ids.begin(); it != trial_ids.end(); ++it)
        {
            id_counter = it->second;
            if (id_counter != key_counter)
            {
                ROS_ERROR("Data does not seem to be complete, id >%i< is missing.", key_counter);
            }
            key_counter++;
        }

        if (key_counter != trial_counts)
        {
            ROS_ERROR("Data does not seem to be complete, there are >%i< files and the trial counter is at >%i<.", key_counter, trial_counts);
            ROS_ERROR("Cannot write to topic name >%s< in directory >%s<.", topic_name.c_str(), path.filename().c_str());
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
        for (int i = 0; i < static_cast<int> (messages.size()) - 1; i++)
        {
            if (messages[i + 1].header.stamp.toSec() - messages[i].header.stamp.toSec() < 1e-6)
            {
                indexes.push_back(i + 1);
            }
            else if (messages[i + 1].header.stamp < ros::TIME_MIN)
            {
                indexes.push_back(i + 1);
            }
        }
        for (std::vector<int>::reverse_iterator rit = indexes.rbegin(); rit != indexes.rend(); ++rit)
        {
            messages.erase(messages.begin() + *rit);
        }
        return true;
    }

    template<typename MessageType>
    inline bool crop(std::vector<MessageType>& messages,
                     const ros::Time& start_time,
                     const ros::Time& end_time)
    {
        // remove messages before start_time
        int initial_index = 0;
        bool found_limit = false;
        for (int i = 0; i < static_cast<int> (messages.size()) && !found_limit; i++)
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
        ROS_ASSERT_MSG(found_limit, "Looks like the start time is not contained in the messages. Check the requested times.");
        messages.erase(messages.begin(), messages.begin() + initial_index);

        int final_index = static_cast<int> (messages.size());
        // remove messages after end_time
        found_limit = false;
        for (int i = static_cast<int> (messages.size()) - 1; i >= 0 && !found_limit; --i)
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
        ROS_ASSERT_MSG(found_limit, "Looks like the end time is not contained in the messages. Check the requested times.");
        messages.erase(messages.begin() + final_index, messages.end());
        return true;
    }

    template<typename MessageType>
    inline bool computeMeanDtAndInputVector(const std::vector<MessageType>& messages,
                                            double& mean_dt,
                                            std::vector<double>& input_vector)
    {
        int num_messages = static_cast<int> (messages.size());
        if (num_messages < 2)
        {
            ROS_ERROR("There should be at least 2 messages, but there are only >%i<.", num_messages);
            return false;
        }

        // compute mean dt of the provided time stamps
        double dts[num_messages - 1];
        mean_dt = 0.0;

        input_vector.clear();
        input_vector.resize(num_messages);
        input_vector[0] = messages[0].header.stamp.toSec();
        for (int i = 0; i < num_messages - 1; i++)
        {
            dts[i] = messages[i + 1].header.stamp.toSec() - messages[i].header.stamp.toSec();
            mean_dt += dts[i];
            input_vector[i + 1] = input_vector[i] + dts[i];
        }
        mean_dt /= static_cast<double> (num_messages - 1);
        return true;
    }    
}

#endif
