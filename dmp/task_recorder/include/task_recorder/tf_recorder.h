/*************************************************************************
	> File Name: tf_recorder.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 26 Jun 2017 03:23:40 PM PDT
 ************************************************************************/

#ifndef _TF_RECORDER_H
#define _TF_RECORDER_Hi

// system includes 
#include <vector>
#include <string>

// ros includes 
#include <tf/tfMessage.h>

// local includes 
#include <task_recorder/task_recorder.h>
#include <task_recorder/DataSample.h>

namespace task_recorder
{
    class TFRecorder : public TaskRecorder<tf::tfMessage>
    {
        public:
            
            TFRecorder(ros::NodeHandle node_handle);
            virtual ~TFRecorder() {};

            bool initialize(const std::string topic_name = std::string("/tf"))
            {
                return TaskRecorder<tf::tfMessage>::initialize(topic_name);
            }

            
            /*!
             * @param transform
             * @param data_sample
             * @return True on success, otherwise False
             */
            bool transformMsg(const tf::tfMessage& transform,
                              task_recorder::DataSample& data_sample);

            /*
             * @return 
             */
            int getNumSignals() const 
            {
                return static_cast<int>(NUM_SIGNALS_PER_TRANSFORM * transform_names_.size());
            }

            std::vector<std::string> getNames() const;

            /*!
             * @return
             */
            static std::string getClassName()
            {
                return std::string("TFRecorder");
            }
        
        private:
            static const int NUM_SIGNALS_PER_TRANSFORM = (3+4);

            int num_transforms_;
            
            std::vector<std::string> transform_names_;
            std::vector<int> indices_;
            
    };
}

#endif
