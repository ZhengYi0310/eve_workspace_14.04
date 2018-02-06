/*************************************************************************
	> File Name: robot_states_publisher.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 12 Jul 2017 10:52:51 AM PDT
 ************************************************************************/

#include<iostream>
#include <algorithm>
#include "barrett_hw/robot_states_publisher.h"

namespace robot_states_publisher 
{
    bool RobotStatesPublisher::init(barrett_model::RobotStateInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
        if (!controller_nh.getParam("publish_rate", publish_rate_))
        {
            ROS_ERROR("Parameter 'publish_rate' not set!");
            return false;
        }

        if (!controller_nh.getParam("tactile", biotac_sensors_exist_))
        {
            ROS_ERROR("Parameter 'tactile' not set!");
            return false;
        }

        const std::vector<std::string>& robot_names = hw->getNames();
        num_robots_ = robot_names.size();
        for (int i = 0; i < num_robots_; i++)
        {
            ROS_INFO("Get a robot with name >>%s<<.", robot_names[i].c_str());
            robot_state_handle_.push_back(hw->getHandle(robot_names[i]));
        }

        //realtime_pub_joint_states_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "joint_states_rt", 10));
        realtime_pub_joint_states_.reset(new realtime_tools::RealtimePublisher<barrett_hw::joint_state>(root_nh, "joint_states_rt", 10));
        const std::vector<std::string>& joint_names = (hw->getHandle(robot_names[0])).getJointStateInterface().getNames();
        num_joints_ = joint_names.size();
        for (int i = 0; i < num_joints_; i++)
        {
            ROS_INFO("The robot has a joint with name >>%s<<.", joint_names[i].c_str());
            joint_state_handles_.push_back((hw->getHandle(robot_names[0])).getJointStateInterface().getHandle(joint_names[i]));

            realtime_pub_joint_states_->msg_.name.push_back(joint_names[i]);
            realtime_pub_joint_states_->msg_.position.push_back(0.0);
            realtime_pub_joint_states_->msg_.velocity.push_back(0.0);
            realtime_pub_joint_states_->msg_.effort.push_back(0.0);
        }
        
        realtime_pub_cartesian_states_.reset(new realtime_tools::RealtimePublisher<barrett_hw::arm_cartesian_state>(root_nh, "cartesian_pose", 10));
        const std::vector<std::string>& cartesian_pose_names = (hw->getHandle(robot_names[0])).getArmPoseStatesInterface().getNames();
        num_cartesian_pose_ = cartesian_pose_names.size();
        for (int i = 0; i < num_cartesian_pose_; i++)
        {
            ROS_INFO("The robot has a base frame with name >>%s<<.", (hw->getHandle(robot_names[0])).getArmPoseStatesInterface().getHandle(cartesian_pose_names[i]).getBaseFrame().c_str());
            arm_cartesian_state_handles_.push_back((hw->getHandle(robot_names[0])).getArmPoseStatesInterface().getHandle(cartesian_pose_names[i]));
        }



        if (biotac_sensors_exist_)
        {
            realtime_pub_biotac_hand_.reset(new realtime_tools::RealtimePublisher<biotac_sensors::BioTacHand>(root_nh, "biotac_sensors", 10));
            biotac_hand_.reset(new biotac::BioTacHandClass("left_hand_biotacs"));
            biotac_hand_->initBioTacSensors();
        }
        
        return true;
    }

    void RobotStatesPublisher::starting(const ros::Time& time)
    {
        last_publish_time_ = time;
        controller_start_time_ = time;
    }

    void RobotStatesPublisher::update(const ros::Time& time, const ros::Duration& /*period*/)
    {
        if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
        {
            last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

            ros::Time duration((time - controller_start_time_).toSec());
            
            if (realtime_pub_joint_states_->trylock())
            {
                realtime_pub_joint_states_->msg_.header_from_controller_start = duration;
                realtime_pub_joint_states_->msg_.header.stamp = time;
                //tf::poseKDLToMsg(arm_cartesian_state_handles_[0].getPose(),  realtime_pub_no_tactile_->msg_.Pose);
                //tf::twistKDLToMsg(arm_cartesian_state_handles_[0].getTwist(), realtime_pub_no_tactile_->msg_.Twist);
                for (int i = 0; i < num_joints_; i++)
                {
                    realtime_pub_joint_states_->msg_.position[i] = joint_state_handles_[i].getPosition();
                    realtime_pub_joint_states_->msg_.velocity[i] = joint_state_handles_[i].getVelocity();
                    realtime_pub_joint_states_->msg_.effort[i] = joint_state_handles_[i].getEffort();
                }
                realtime_pub_joint_states_->unlockAndPublish();                
            }

            if (realtime_pub_cartesian_states_->trylock())
            {
                //realtime_pub_cartesian_states_->msg_.header.stamp = duration;
                realtime_pub_cartesian_states_->msg_.header.stamp = time;
                realtime_pub_cartesian_states_->msg_.header_from_controller_start = duration;
                realtime_pub_cartesian_states_->msg_.base_frame = arm_cartesian_state_handles_[0].getBaseFrame();
                tf::poseKDLToMsg(arm_cartesian_state_handles_[0].getPose(),  realtime_pub_cartesian_states_->msg_.Pose);
                tf::twistKDLToMsg(arm_cartesian_state_handles_[0].getTwist(), realtime_pub_cartesian_states_->msg_.Twist);
                realtime_pub_cartesian_states_->unlockAndPublish();
            }

            if (biotac_sensors_exist_) 
            {
                if (realtime_pub_biotac_hand_->trylock())
                {
                    realtime_pub_biotac_hand_->msg_ = biotac_hand_->collectBatch();
                    realtime_pub_biotac_hand_->msg_.frame_collection_from_header = ros::Time((time - realtime_pub_biotac_hand_->msg_.bt_time.frame_start_time).toSec()); // frame collection starts earlier than the header of this loop
                    realtime_pub_biotac_hand_->msg_.header_from_controller_start = ros::Time((time - controller_start_time_).toSec());
                    realtime_pub_biotac_hand_->msg_.header.stamp = time;
                    realtime_pub_biotac_hand_->msg_.bt_time.frame_start_time = ros::Time((realtime_pub_biotac_hand_->msg_.bt_time.frame_start_time - controller_start_time_).toSec());
                    realtime_pub_biotac_hand_->msg_.bt_time.frame_end_time = ros::Time((realtime_pub_biotac_hand_->msg_.bt_time.frame_end_time - controller_start_time_).toSec());                                   
                    realtime_pub_biotac_hand_->unlockAndPublish();
                }                 
            }
        }
    }

    void RobotStatesPublisher::stopping(const ros::Time& /*time*/)
    {}
}
PLUGINLIB_EXPORT_CLASS(robot_states_publisher::RobotStatesPublisher, controller_interface::ControllerBase)

