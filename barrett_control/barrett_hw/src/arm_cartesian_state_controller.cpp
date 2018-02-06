/*************************************************************************
	> File Name: arm_cartesian_state_controller.cpp
	> Author: Yi Zheng
	> Mail: hczhengcq@gmail.com
	> Created Time: Sat 08 Jul 2017 11:20:21 AM PDT
 ************************************************************************/
#include<iostream>
#include <barrett_hw/arm_cartesian_state_controller.h>

namespace arm_cartesian_state_controller 
{
    bool ArmCartesianStateController::init(barrett_model::ArmPoseStatesInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
        // get all prudct type names from the hardware interface
        const std::vector<std::string>& product_names = hw->getNames();
        num_devices_ = product_names.size();
        for (size_t i = 0; i < num_devices_; i++)
        {
            ROS_INFO("Get a deivce with type >>%s<< with root link >>%s<<.", product_names[i].c_str(), hw->getHandle(product_names[i]).getBaseFrame().c_str());
        }

        // get the publish period 
        if (!controller_nh.getParam("publish_rate", publish_rate_))
        {
            ROS_ERROR("Parameter 'publish_rate' not set!");
            return false;
        }
        /*
        if (!controller_nh.getParam("visualization", visualization_))
        {
            ROS_ERROR("Parameter 'visualization' not set!");
            return false;
        }
        */

        // realtime publisher 
        realtime_pub_.reset(new realtime_tools::RealtimePublisher<barrett_hw::robot_cartesian_state>(root_nh, "robot_cartesian_state", 10));
        //visualization_realtime_pub_.reset(new realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray>(root_nh, "robot_cartesian_state_visualization", 10));
        // allocate messages  get the topname space
        for (size_t i = 0; i < num_devices_; i++)
        {
            arm_cartesian_state_handle_.push_back(hw->getHandle(product_names[i]));
            barrett_hw::arm_cartesian_state arm_cartesian_state;
            realtime_pub_->msg_.robot_cartesian_state.push_back(arm_cartesian_state);
            robot_cartesian_state_.robot_cartesian_state.push_back(arm_cartesian_state);
            
            /*
            if (visualization_)
            {
                visualization_msgs::Marker marker;
                visualization_realtime_pub_->msg_.markers.push_back(marker);
                visualization_realtime_pub_->msg_.markers[i].header.frame_id = arm_cartesian_state_handle_[i].getBaseFrame();
                std::stringstream ss;
                ss << i;
                visualization_realtime_pub_->msg_.markers[i].ns = arm_cartesian_state_handle_[i].getName() + ss.str();
                visualization_realtime_pub_->msg_.markers[i].id = i;
                visualization_realtime_pub_->msg_.markers[i].type = visualization_msgs::Marker::ARROW;
                visualization_realtime_pub_->msg_.markers[i].action = visualization_msgs::Marker::ADD;
                visualization_realtime_pub_->msg_.markers[i].lifetime = ros::Duration(0);
                visualization_realtime_pub_->msg_.markers[i].frame_locked = false;
                visualization_realtime_pub_->msg_.markers[i].color.r = 0.5f;
                visualization_realtime_pub_->msg_.markers[i].color.g = 0.5f;
                visualization_realtime_pub_->msg_.markers[i].color.b = 1.0f;
                visualization_realtime_pub_->msg_.markers[i].color.a = 0.5;
            }
            */
        }
        return true;
    }

    void ArmCartesianStateController::starting(const ros::Time& time)
    {
        last_publish_time_ = time;
    }

    void ArmCartesianStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
    {
        if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
        {
            // try to publish
            if (realtime_pub_->trylock())
            {
                // Increment the time since we are acutally publishing 
                last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
                // populate the robot cartesian state, i.e. indices [0, num_devices_)
                realtime_pub_->msg_.stamp = time;
                for (size_t i = 0; i < num_devices_; i++)
                {
                    tf::poseKDLToMsg(arm_cartesian_state_handle_[i].getPose(), pose_);
                    tf::twistKDLToMsg(arm_cartesian_state_handle_[i].getTwist(), twist_);
                    robot_cartesian_state_.robot_cartesian_state[i].name = arm_cartesian_state_handle_[i].getName();
                    robot_cartesian_state_.robot_cartesian_state[i].base_frame = arm_cartesian_state_handle_[i].getBaseFrame();
                    robot_cartesian_state_.robot_cartesian_state[i].Twist = twist_;
                    robot_cartesian_state_.robot_cartesian_state[i].Pose = pose_;
                    realtime_pub_->msg_.robot_cartesian_state[i] = robot_cartesian_state_.robot_cartesian_state[i];
                    /*
                    if (visualization_)
                    {
                        visualization_realtime_pub_->msg_.markers[i].header.stamp = time;
                        visualization_realtime_pub_->msg_.markers[i].scale.x = 0.8 * twist_.linear.x;
                        visualization_realtime_pub_->msg_.markers[i].scale.y = 0.8 * twist_.linear.y;
                        visualization_realtime_pub_->msg_.markers[i].scale.z = 0.8 * twist_.linear.z;
                        visualization_realtime_pub_->msg_.markers[i].pose.position.x = pose_.position.x;
                        visualization_realtime_pub_->msg_.markers[i].pose.position.y = pose_.position.y;
                        visualization_realtime_pub_->msg_.markers[i].pose.position.z = pose_.position.z;
                        visualization_realtime_pub_->msg_.markers[i].pose.orientation.x = pose_.orientation.x;
                        visualization_realtime_pub_->msg_.markers[i].pose.orientation.y = pose_.orientation.y;
                        visualization_realtime_pub_->msg_.markers[i].pose.orientation.z = pose_.orientation.z;
                        visualization_realtime_pub_->msg_.markers[i].pose.orientation.w = pose_.orientation.w;
                    }
                    */
                }
                realtime_pub_->unlockAndPublish();
                /*
                if (visualization_)
                {
                    visualization_realtime_pub_->unlockAndPublish();
                }
                */
            }
        }
    }

    void ArmCartesianStateController::stopping(const ros::Time& /*time*/)
    {}
}
PLUGINLIB_EXPORT_CLASS(arm_cartesian_state_controller::ArmCartesianStateController, controller_interface::ControllerBase)


