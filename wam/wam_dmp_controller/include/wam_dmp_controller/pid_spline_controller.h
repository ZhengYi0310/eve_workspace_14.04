/*************************************************************************
	> File Name: joint_space_spline_controller.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 01 Mar 2018 10:05:00 AM PST
 ************************************************************************/

#ifndef _JOINT_SPACE_SPLINE_CONTROLLER_H
#define _JOINT_SPACE_SPLINE_CONTROLLER_H
#include "wam_dmp_controller/pid_controller.h"
#include <wam_dmp_controller/JointPosSpline.h>
#include <wam_dmp_controller/JointPosSplineMsg.h>
#include <wam_dmp_controller/GoHomeSpline.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include "wam_dmp_controller/quintic_spline_segment.h"
#include "wam_dmp_controller/pos_vel_acc_state.h"

namespace wam_dmp_controller
{
    class PIDSplineController : public wam_dmp_controller::PIDController
    {
        public:
            PIDSplineController();
            ~PIDSplineController();

            bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  			void starting(const ros::Time& time);
  			void update(const ros::Time& time, const ros::Duration& period);

         private:
            void set_default_pos_traj();
            bool set_cmd_traj_spline_srv(wam_dmp_controller::JointPosSpline::Request &req, 
                                         wam_dmp_controller::JointPosSpline::Response &res);
            bool get_cmd_traj_spline_srv(wam_dmp_controller::JointPosSpline::Request &req,
                                         wam_dmp_controller::JointPosSpline::Response &res);
            bool go_home_traj_spline_srv(wam_dmp_controller::GoHomeSpline::Request &req,
                                         wam_dmp_controller::GoHomeSpline::Response &res);
            bool set_gains(wam_dmp_controller::SetJointGains::Request &req,
                           wam_dmp_controller::SetJointGains::Response &res);
            bool get_gains(wam_dmp_controller::GetJointGains::Request &req,
                           wam_dmp_controller::GetJointGains::Response &res);

            //void get_parameters(ros::NodeHandle &n);
            void eval_current_point_to_point_traj(const ros::Duration& period,
                                                  Eigen::VectorXd& q_des, 
                                                  Eigen::VectorXd& q_dot_des, 
                                                  Eigen::VectorXd& q_dotdot_des);

            void eval_point_to_point_traj_constants(Eigen::VectorXd& desired_pos,
                                                    double duration);

            ros::ServiceServer set_cmd_traj_pos_service_;     
            ros::ServiceServer get_cmd_traj_pos_service_;
            ros::ServiceServer go_home_traj_service_;
		    ros::ServiceServer set_gains_service_;
            ros::ServiceServer get_gains_service_;
        

            Eigen::VectorXd q_des_;
            Eigen::VectorXd q_dot_des_;
            Eigen::VectorXd q_dotdot_des_;
            Eigen::VectorXd curr_command_;

            double p2p_traj_spline_duration_;
            Eigen::MatrixXf p2p_traj_const_;
            Eigen::VectorXd prev_setpoint_;
            //PosVelAccState<double> prev_setpoint_state_;
            //PosVelAccState<double> curr_setpoint_state_;
            boost::scoped_ptr<PosVelAccState<double> > prev_setpoint_state_;
            boost::scoped_ptr<PosVelAccState<double> > curr_setpoint_state_;
            boost::scoped_ptr<QuinticSplineSegment<double> > spline_seg_;
            double time_;

            boost::mutex p2p_traj_mutex_;
    };
}
#endif