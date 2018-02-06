/*************************************************************************
	> File Name: robot_states_publisher.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 11 Jul 2017 10:35:51 PM PDT
 ************************************************************************/

#ifndef _ROBOT_STATES_PUBLISHER_H
#define _ROBOT_STATES_PUBLISHER_H

#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <kdl_conversions/kdl_msg.h> //watch out for migration for a newer version ROS

#include <boost/shared_ptr.hpp>

#include <barrett_hw/arm_cartesian_state.h>
#include <barrett_model/robot_state_interface.h>
#include <barrett_hw/joint_state.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <biotac_sensors/biotac_hand_class.h>
#include <biotac_sensors/BioTacHand.h>
#include <biotac_sensors/BioTacData.h>
#include <biotac_sensors/BioTacTime.h>

namespace robot_states_publisher
{
    /**
     * \brief publisher that publishes the state of the robot arm
     */
    class RobotStatesPublisher : public controller_interface::Controller<barrett_model::RobotStateInterface>
    {
        public:
            RobotStatesPublisher() : publish_rate_(0.0) {};
            virtual bool init(barrett_model::RobotStateInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
            virtual void starting(const ros::Time& time);
            virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
            virtual void stopping(const ros::Time& /*time*/);
        private:
            boost::shared_ptr<barrett_model::RobotStateInterface> robot_state_;

            //boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_pub_joint_states_;
            boost::shared_ptr<realtime_tools::RealtimePublisher<barrett_hw::joint_state> > realtime_pub_joint_states_;
            boost::shared_ptr<realtime_tools::RealtimePublisher<barrett_hw::arm_cartesian_state> > realtime_pub_cartesian_states_;
            boost::shared_ptr<realtime_tools::RealtimePublisher<biotac_sensors::BioTacHand> > realtime_pub_biotac_hand_;

            boost::shared_ptr<biotac::BioTacHandClass> biotac_hand_;
            boost::shared_ptr<sensor_msgs::JointState> joint_states_;
            boost::shared_ptr<geometry_msgs::Pose> cartesian_states_;

            bool biotac_sensors_exist_;            
            int num_robots_;
            int num_joints_;
            int num_cartesian_pose_;
            ros::Time last_publish_time_;
            double publish_rate_;
            ros::Time controller_start_time_;            
            geometry_msgs::Twist twist_;
            geometry_msgs::Pose pose_;
            std::vector<barrett_model::ArmPoseStatesHandle> arm_cartesian_state_handles_;
            std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
            std::vector<barrett_model::RobotStateHandle> robot_state_handle_;
            
            
            
    };
}
#endif
