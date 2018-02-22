/*************************************************************************
	> File Name: joint_impedance_controller.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 21 Feb 2018 04:22:13 PM PST
 ************************************************************************/


#ifndef __WAM_CONTROLLER_JOINT_INPEDANCE_CONTROLLER_H
#define __WAM_CONTROLLER_JOINT_INPEDANCE_CONTROLLER_H

#include "KinematicChainControllerBase.h"
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>

/*
	tau_cmd_ = Kp_*(q_des_ - q_msr_) + kd_*dotq_msr_ + G(q_msr_)
*/

namespace wam_dmp_controller
{
    
	class JointImpedanceController : public wam_dmp_controller::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	    public:
            struct Commands
            {
                std::vector<double> positions_; // Last commanded position 
            };
        

		    JointImpedanceController();
		    ~JointImpedanceController();

		    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

		    void starting(const ros::Time& time);

		    void update(const ros::Time& time, const ros::Duration& period);
		    void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
		    void setParam(const std_msgs::Float64MultiArray::ConstPtr &msg, KDL::JntArray* array, std::string s);
        
	    private:

		    ros::Subscriber sub_stiffness_, sub_damping_, sub_add_torque_;
		    ros::Subscriber sub_posture_;

		    KDL::JntArray q_des_;
		    KDL::JntArray tau_des_;
		    KDL::JntArray Kp_, Kd_;
            Commands command_struct_;
            realtime_tools::RealtimeBuffer<Commands> command_buffer_;
            std::vector<double> command_vec_;
	};

} // namespace

#endif
