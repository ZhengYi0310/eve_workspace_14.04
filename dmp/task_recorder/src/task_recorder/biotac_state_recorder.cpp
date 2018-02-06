/*************************************************************************
	> File Name: biotac_state_recorder.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 24 Jul 2017 10:06:02 AM PDT
 ************************************************************************/

// system includes
#include <sstream>
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes 
#include <task_recorder_utilities/task_recorder_utilities.h>
#include <task_recorder/biotac_state_recorder.h>

namespace task_recorder
{
    BioTacStateRecorder::BioTacStateRecorder(ros::NodeHandle node_handle) : TaskRecorder<biotac_sensors::BioTacHand>(node_handle)
    {}

    bool BioTacStateRecorder::transformMsg(const biotac_sensors::BioTacHand& biotac_states, task_recorder::DataSample& data_sample)
    {
        data_sample.header = biotac_states.header;
        num_biotac_sensors_ = biotac_states.bt_data.size();
        biotac_serials_.resize(num_biotac_sensors_);
        indices_.resize(num_biotac_sensors_);
        
        for (int i = 0; i < biotac_states.bt_data.size(); i++)
        {
            indices_[i] = biotac_states.bt_data[i].bt_position;

            // fill out the data sample
            data_sample.data[TDC_INDEX + i * TOTAL_INDEX] = (double)biotac_states.bt_data[i].tdc_data;
            data_sample.data[TAC_INDEX + i * TOTAL_INDEX] = (double)biotac_states.bt_data[i].tac_data;
            data_sample.data[PDC_INDEX + i * TOTAL_INDEX] = (double)biotac_states.bt_data[i].pdc_data;

            for (int j = 1; j <= PAC_INDEX; j++)
            {
                data_sample.data[PDC_INDEX + j + i * TOTAL_INDEX] = (double)biotac_states.bt_data[i].pac_data[j - 1];
            }

            for (int j = 1; j <= ELEC_INDEX; j++)
            {
                data_sample.data[PDC_INDEX + PAC_INDEX + j + i * TOTAL_INDEX] = (double)biotac_states.bt_data[i].electrode_data[j - 1];
            }
        }
        data_sample.data[num_biotac_sensors_ * TOTAL_INDEX] = biotac_states.header_from_controller_start.toSec();
        data_sample.data[num_biotac_sensors_ * TOTAL_INDEX + 1] = biotac_states.frame_collection_from_header.toSec();
        
        return true;
    }

    std::vector<std::string> BioTacStateRecorder::getNames() const 
    {
        std::vector<std::string> names;

        if (indices_.size() != 0)
        {
            for (int i = 0; i < num_biotac_sensors_; i++)
            {
                std::stringstream ss_pos;
                ss_pos << indices_[i];
                names.push_back("position " + ss_pos.str() + "_tdc_data");
                names.push_back("position " + ss_pos.str() + "_tac_data");
                names.push_back("position " + ss_pos.str() + "_pdc_data");
            
                for (int j = 0; j < PAC_INDEX; j++)
                {
                    std::stringstream ss;
                    ss << j;
                    names.push_back("position " + ss_pos.str() + "_pac_data_" + ss.str());
                }

                for (int j = 0; j < ELEC_INDEX; j++)
                {
                    std::stringstream ss;
                    ss << j;
                    names.push_back("position " + ss_pos.str() + "_elec_data_" + ss.str());
                }
            }
            names.push_back("header_from_controller_start");
            names.push_back("frame_collection_from_header");
        }
        else 
        {
            names.reserve(100);
        }
        return names;
    }
    
}

