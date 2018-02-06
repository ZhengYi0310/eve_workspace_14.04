/*************************************************************************
	> File Name: arm_cartesian_state_controller.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 07 Jul 2017 04:27:04 PM PDT
 ************************************************************************/

#ifndef _CARTESIAN_STATE_CONTROLLER_H
#define _CARTESIAN_STATE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>

#include <barrett_model/arm_pose_interface.h>
#include <barrett_hw/arm_cartesian_state.h>
#include <barrett_hw/robot_cartesian_state.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <kdl_conversions/kdl_msg.h> //watch out for migration for a newer version ROS

namespace arm_cartesian_state_controller
{
    /**
     * \brief Controller that publishes the cartesian state of an Wam arm.
     *
     * This controller publishes the state of all resources registered to a \c barrett_model::arm_pose_interface to a
     * topic of type \c barrett_hw/arm_cartesian_state. The following is a basic configuration of the controller.
     *
     * \code
     * biotac_state_controller:
     *   type: barrett_model/BioTacStateController
     *   publish_rate: 50
     *   TODO perhaps add visualization markers later
     * \endcode
     */   
    class ArmCartesianStateController : public controller_interface::Controller<barrett_model::ArmPoseStatesInterface>
    {
        public:
            ArmCartesianStateController() : publish_rate_(0) {};
            virtual bool init(barrett_model::ArmPoseStatesInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
            virtual void starting(const ros::Time& time);
            virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
            virtual void stopping(const ros::Time& /*time*/);

        private:
        std::vector<barrett_model::ArmPoseStatesHandle> arm_cartesian_state_handle_;
        boost::shared_ptr<realtime_tools::RealtimePublisher<barrett_hw::robot_cartesian_state> > realtime_pub_;
        //boost::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray> > visualization_realtime_pub_;
        ros::Time last_publish_time_;
        double publish_rate_;
        //bool visualization_;
        unsigned int num_devices_;
        geometry_msgs::Twist twist_;
        geometry_msgs::Pose pose_;
        barrett_hw::robot_cartesian_state robot_cartesian_state_;
    };
}

#endif
