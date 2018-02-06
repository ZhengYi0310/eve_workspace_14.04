/*************************************************************************
	> File Name: biotac_state_controller.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 06 Jul 2017 10:58:27 AM PDT
 ************************************************************************/

#include <algorithm>
#include <cstddef>

#include "barrett_hw/biotac_state_controller.h"

namespace biotac_state_controller 
{
    bool BioTacStateController::init(hardware_interface::JointStateInterface* /*hw*/, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
        // get the publish period 
        if (!controller_nh.getParam("publish_rate", publish_rate_))
        {
            ROS_ERROR("Parameter 'publish_rate' not set!");
            return false;
        }

        biotac_hand_.reset(new biotac::BioTacHandClass("left_hand_biotacs")); 
        realtime_pub_.reset(new realtime_tools::RealtimePublisher<biotac_sensors::BioTacHand>(root_nh, "biotac_states", 4));

        biotac_hand_->initBioTacSensors();
        return true;
    }

    void BioTacStateController::starting(const ros::Time& time)
    {
        //initialize time 
        last_publish_time_ = time;
    }

    void BioTacStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
    {
        // limit the rate of publishing 
        if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
        {
            // try to publish 
            if (realtime_pub_->trylock())
            {
                // increment time since we're actually publishing
                last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
                realtime_pub_->msg_ = biotac_hand_->collectBatch();
                realtime_pub_->unlockAndPublish();
            }
        }
    }

    void BioTacStateController::stopping(const ros::Time& /*time*/)
    {}
}
PLUGINLIB_EXPORT_CLASS(biotac_state_controller::BioTacStateController, controller_interface::ControllerBase)

