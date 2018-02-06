/*************************************************************************
	> File Name: biotac_state_recorder.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 20 Jul 2017 04:55:24 PM PDT
 ************************************************************************/

#ifndef _BIOTAC_STATE_RECORDER_H
#define _BIOTAC_STATE_RECORDER_H

// system includes 
#include <biotac_sensors/BioTacHand.h>

// local includes 
#include <task_recorder/task_recorder.h>
#include <task_recorder/DataSample.h>

namespace task_recorder
{
    class BioTacStateRecorder : public TaskRecorder<biotac_sensors::BioTacHand>
    {
        public:

            /*!
             */
            BioTacStateRecorder(ros::NodeHandle node_handle);
            virtual ~BioTacStateRecorder() {};

            bool initialize(const std::string topic_name = std::string("/biotac_sensors"))
            {
                return TaskRecorder<biotac_sensors::BioTacHand>::initialize(topic_name);
            }

            /*
             * @param biotac_states
             * @param data_sample 
             * @return True on success, otherwise False 
             */
            bool transformMsg(const biotac_sensors::BioTacHand& biotac_states, task_recorder::DataSample& data_sample);

            /*!
             * return 
             */
            int getNumSignals() const 
            {
                return static_cast<int>(1 * (TAC_INDEX + PDC_INDEX + PAC_INDEX + ELEC_INDEX) + 2);
            }
            
            /*!
             * @return 
             */
            std::vector<std::string> getNames() const;

            /*!
             * @return 
             */
            static std::string getClassName()
            {
                return std::string("BioTacStateRecorder");
            }

        private:
            int num_biotac_sensors_;
            std::vector<std::string> biotac_serials_;
            std::vector<int> indices_;

            enum
            {
                TDC_INDEX = 0, TAC_INDEX, PDC_INDEX, PAC_INDEX = 22, ELEC_INDEX = 19
            };

            static const unsigned int TOTAL_INDEX = TAC_INDEX + PDC_INDEX + PAC_INDEX + ELEC_INDEX;
    };
}
#endif
