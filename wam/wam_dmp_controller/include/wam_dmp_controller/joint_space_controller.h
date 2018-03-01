/*************************************************************************
	> File Name: joint_space_controller.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 21 Feb 2018 08:27:45 PM PST
 ************************************************************************/

#ifndef __WAM_DMP_CONTROLLER_JOINT_SPACE_CONTROLLER_H
#define __WAM_DMP_CONTROLLER_JOINT_SPACE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <std_msgs/Float64MultiArray.h>
#include <wam_dmp_controller/SetJointPos.h>
#include <wam_dmp_controller/SetJointGains.h>
#include <wam_dmp_controller/GetJointPos.h>
#include <wam_dmp_controller/GetJointGains.h>
#include <wam_dmp_controller/GoHome.h>

#include <boost/scoped_ptr.hpp>
/*
	tau_cmd_ = M(q_msr_)[Kp_*(q_des_ - q_msr_) + Kv_*(dotq_des - dotq_msr_)] + C(q_msr_, dotq_msr_)q_msr_ + G(q_msr_)
*/

namespace wam_dmp_controller
{
	class JointSpaceController : public wam_dmp_controller::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	    public:

		    JointSpaceController();
		    ~JointSpaceController();

		    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		    void starting(const ros::Time& time);
		    void update(const ros::Time& time, const ros::Duration& period);
            
            struct Commands
            {
                std::vector<double> positions_;
            };
	    private:
            bool go_home(wam_dmp_controller::GoHome::Request &req,
                         wam_dmp_controller::GoHome::Response &res);
		    bool set_joint_pos(wam_dmp_controller::SetJointPos::Request &req,
                               wam_dmp_controller::SetJointPos::Response &res);
            bool get_joint_pos(wam_dmp_controller::GetJointPos::Request &req,
                               wam_dmp_controller::GetJointPos::Response &res);
		    bool set_gains(wam_dmp_controller::SetJointGains::Request &req,
                           wam_dmp_controller::SetJointGains::Response &res);
            bool get_gains(wam_dmp_controller::GetJointGains::Request &req,
                           wam_dmp_controller::GetJointGains::Response &res);

            void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
           
            ros::Subscriber sub_posture_;
		    ros::ServiceServer set_posture_service_;
            ros::ServiceServer get_posture_service_;
		    ros::ServiceServer set_gains_service_;
            ros::ServiceServer get_gains_service_;
            ros::ServiceServer go_home_service_;
        
		    KDL::JntArray cmd_states_;
		    int cmd_flag_;	// discriminate if a user command arrived
		    double lambda;	// flattening coefficient of tanh
		    int step_;		// step used in tanh for reaching gradually the desired posture
		    KDL::JntArray joint_initial_states_; // joint as measured at the beginning of the control action
		    KDL::JntArray current_cmd_; // command value as delta to be added to joint_initial_states_

		    KDL::JntArray tau_cmd_;
		    KDL::JntSpaceInertiaMatrix M_; //Inertia matrix
		    KDL::JntArray C_, G_;	//Coriolis and Gravitational matrices
		    KDL::JntArray Kp_, Kv_;	//Position and Velocity gains
            KDL::JntArray home_;

		    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;

            Commands command_struct_;
            realtime_tools::RealtimeBuffer<Commands> command_buffer_;

	    };
}

#endif
