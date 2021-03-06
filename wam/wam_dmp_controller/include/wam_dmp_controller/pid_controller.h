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
#include <wam_dmp_controller/SetJointPosMsg.h>
#include <wam_dmp_controller/SetJointPosStampedMsg.h>
#include <wam_dmp_controller/SetJointGains.h>
#include <wam_dmp_controller/GetJointPos.h>
#include <wam_dmp_controller/GetJointGains.h>
#include <wam_dmp_controller/GoHome.h>
#include <control_toolbox/pid.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

/*
	tau_cmd_ = M(q_msr_)[Kp_*(q_des_ - q_msr_) + Kv_*(dotq_des - dotq_msr_)] + C(q_msr_, dotq_msr_)q_msr_ + G(q_msr_)
*/

namespace wam_dmp_controller
{
	class PIDController : public wam_dmp_controller::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
        friend class PIDSplineController;
	    public:

		    PIDController();
		    ~PIDController();

		    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		    void starting(const ros::Time& time);
		    void update(const ros::Time& time, const ros::Duration& period);
            
            struct Commands
            {
                std::vector<double> positions_;
                std::vector<double> velocities_;
                std::vector<double> accelerations_;
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
            void setCommandRT(Eigen::VectorXd& q_des, Eigen::VectorXd& q_dot_des, Eigen::VectorXd& q_dotdot_des);

            void command(const std_msgs::Float64MultiArray::ConstPtr &msg);

            void publish_(const ros::Time& time);

            ros::Time last_publish_time_;
            double publish_rate_;
            double publish_counter_;
           
            ros::Subscriber sub_posture_;
		    ros::ServiceServer set_posture_service_;
            ros::ServiceServer get_posture_service_;
		    //ros::ServiceServer set_gains_service_;
            //ros::ServiceServer get_gains_service_;
            ros::ServiceServer go_home_service_;

            std::vector<control_toolbox::Pid> pid_controllers_;
        
		    KDL::JntArray cmd_states_;
		    int cmd_flag_;	// discriminate if a user command arrived
		    double lambda;	// flattening coefficient of tanh
		    int step_;		// step used in tanh for reaching gradually the desired posture
		    KDL::JntArray joint_initial_states_; // joint as measured at the beginning of the control action
		    KDL::JntArray current_cmd_; // command value as delta to be added to joint_initial_states_

		    KDL::JntArray tau_cmd_;
		    KDL::JntSpaceInertiaMatrix M_; //Inertia matrix
		    KDL::JntArray C_, G_;	//Coriolis and Gravitational matrices
		    KDL::JntArray Kp_, Kv_, Ki_;	//Position and Velocity gains
            KDL::JntArray home_;
            KDL::JntArray last_pos_error_, last_vel_error_;

		    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;

            Commands command_struct_;
            realtime_tools::RealtimeBuffer<Commands> command_buffer_;
            boost::scoped_ptr<realtime_tools::RealtimePublisher<wam_dmp_controller::SetJointPosStampedMsg> > pub_q_des_, pub_qdot_des_, pub_qdotdot_des_, pub_q_err_;
	    };
}

#endif
