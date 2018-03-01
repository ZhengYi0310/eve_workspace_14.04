/*************************************************************************
	> File Name: joint_space_spline_controller.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 01 Mar 2018 10:24:53 AM PST
 ************************************************************************/

#include<iostream>
#include <pluginlib/class_list_macros.h>
#include <wam_dmp_controller/joint_space_spline_controller.h>
#include <math.h>
#include <angles/angles.h>
#include <geometry_msgs/WrenchStamped.h>
#define DEFAULT_P2P_TRAJ_DURATION 10.0
#define P2P_COEFF_3 10.0
#define P2P_COEFF_4 -15.0 
#define P2P_COEFF_5 6.0

namespace wam_dmp_controller
{
    
    JointSpaceSplineController::JointSpaceSplineController() {}
    JointSpaceSplineController::~JointSpaceSplineController() {}

    bool JointSpaceSplineController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
    {
        JointSpaceController::init(hw, n);

        set_cmd_traj_pos_service_ = n.advertiseService("set_traj_pos_cmd",
                                                       &JointSpaceSplineController::set_cmd_traj_spline_srv, this);
        get_cmd_traj_pos_service_ = n.advertiseService("set_traj_pos_cmd",
                                                       &JointSpaceSplineController::get_cmd_traj_spline_srv, this);

        // set trajectory duration
        p2p_traj_spline_duration_ = DEFAULT_P2P_TRAJ_DURATION;

        p2p_traj_const_ = Eigen::MatrixXf(4, kdl_chain_.getNrOfJoints());

        q_des_         = Eigen::VectorXd(kdl_chain_.getNrOfJoints());
        q_dot_des_     = Eigen::VectorXd(kdl_chain_.getNrOfJoints());
        q_dotdot_des_  = Eigen::VectorXd(kdl_chain_.getNrOfJoints());
        curr_command_  = Eigen::VectorXd(kdl_chain_.getNrOfJoints());
        prev_setpoint_ = Eigen::VectorXd(kdl_chain_.getNrOfJoints());
        q_des_.setZero();
        q_dot_des_.setZero();
        q_dotdot_des_.setZero();
        curr_command_.setZero();
        prev_setpoint_.setZero();

        return true;
    }

    void JointSpaceSplineController::starting(const ros::Time& time)
    {
        JointSpaceController::starting(time);
        /*
            Important!!!!!!!!!!!!!!!!!!!
            whenever the controller is started the set point is set
            to the current configuration (For safety!!!!!!!!!)
            !!!!!!!!!!!!!!!!!!!!!!!!!
        */

        // set default trajectory 
        set_default_pos_traj(); 
    }

    void JointSpaceSplineController::update(const ros::Time& time, const ros::Duration& period)
    {
        Eigen::VectorXd q_des(kdl_chain_.getNrOfJoints());
        Eigen::VectorXd qdot_des(kdl_chain_.getNrOfJoints());
        Eigen::VectorXd qdotdot_des(kdl_chain_.getNrOfJoints());
        eval_current_point_to_point_traj(period, q_des_, q_dot_des_, q_dotdot_des_);
        JointSpaceController::setCommandRT(q_des_, q_dot_des_, q_dotdot_des_);
        JointSpaceController::update(time, period);
    }

    void JointSpaceSplineController::set_default_pos_traj()
    {
        // get current robot joints configuration q
        KDL::JntArray q;
        q.resize(kdl_chain_.getNrOfJoints());
        for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
            q(i) = joint_handles_[i].getPosition();
    
        // set position and attitude tajectory constants
        for(int i=0; i<p2p_traj_const_.cols(); i++)
        {
	        p2p_traj_const_(1, i) = 0;
	        p2p_traj_const_(2, i) = 0;
	        p2p_traj_const_(3, i) = 0;
        }

        for(int i=0; i<p2p_traj_const_.cols(); i++)
        {
            p2p_traj_const_(0, i) = q.data[i];
            prev_setpoint_(i) = q.data[i];
        }
        // reset the time
        time_ = p2p_traj_spline_duration_;

        p2p_traj_mutex_.unlock();
    }


    void JointSpaceSplineController::eval_current_point_to_point_traj(const ros::Duration& period,
                                                                       Eigen::VectorXd& q_des, 
                                                                       Eigen::VectorXd& q_dot_des, 
                                                                       Eigen::VectorXd& q_dotdot_des)
    {
        p2p_traj_mutex_.lock();
        time_ += period.toSec();

        if (time_ > p2p_traj_spline_duration_)
        {    
            time_ = p2p_traj_spline_duration_;
            //run_spline_ = false;
        }

        for (int i=0; i<3; i++)
        {
	        q_des[i] = p2p_traj_const_(0, i) + p2p_traj_const_(1, i) * pow(time_, 3) + \
                       p2p_traj_const_(2, i) * pow(time_, 4) + p2p_traj_const_(3, i) * pow(time_, 5);

	        q_dot_des[i] = 3 * p2p_traj_const_(1, i) * pow(time_, 2) + \
                          4 * p2p_traj_const_(2, i) * pow(time_, 3) + 5 * p2p_traj_const_(3, i) * pow(time_, 4);

	        q_dotdot_des[i] = 3 * 2 *  p2p_traj_const_(1, i) * time_ + \
                             4 * 3 * p2p_traj_const_(2, i) * pow(time_, 2) + 5 * 4 * p2p_traj_const_(3, i) * pow(time_, 3);
        }
        p2p_traj_mutex_.unlock();
    }

    void JointSpaceSplineController::eval_point_to_point_traj_constants(Eigen::VectorXd& desired_pos,
                                                                        double duration)
    {
         // evaluate common part of constants
        double constant_0, constant_1, constant_2;
        constant_0 = P2P_COEFF_3 / pow(duration, 3);
        constant_1 = P2P_COEFF_4 / pow(duration, 4);
        constant_2 = P2P_COEFF_5 / pow(duration, 5);

        // evaluate constants for x and y trajectories
        for (int i=0; i<kdl_chain_.getNrOfJoints(); i++)
        {
            double error = angles::normalize_angle(desired_pos(i) - prev_setpoint_(i));
	        p2p_traj_const_(0, i) = prev_setpoint_(i);
	        p2p_traj_const_(1, i) = error * constant_0;
	        p2p_traj_const_(2, i) = error * constant_1;
	        p2p_traj_const_(3, i) = error * constant_2;
        }
        prev_setpoint_ = desired_pos;
    }

    bool JointSpaceSplineController::set_cmd_traj_spline_srv(wam_dmp_controller::JointPosSpline::Request &req, 
                                                             wam_dmp_controller::JointPosSpline::Response &res)
    {
        if (command_struct_.positions_.size() != req.command.joint_pos.size())
        {
            ROS_ERROR("command struct has size %lu, request command has size %u, they are not equal", 
                      command_struct_.positions_.size(), 
                      req.command.joint_pos.size());
        }
        if (time_ < p2p_traj_spline_duration_)
        {
            res.command.elapsed_time = time_;
            res.command.accepted = false;
            res.command.p2p_traj_duration = p2p_traj_spline_duration_;
            
            return true;
        }
        res.command.accepted = true;

        for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
        {
            command_struct_.positions_[i] = req.command.joint_pos[i];
        }
        curr_command_ = Eigen::Map<Eigen::VectorXd>(&command_struct_.positions_[0], command_struct_.positions_.size());
        p2p_traj_mutex_.lock();

        p2p_traj_spline_duration_ = req.command.p2p_traj_duration;

        //command_buffer_.writeFromNonRT(command_struct_); Don't do this when set a spline!!!!!! 
        eval_point_to_point_traj_constants(curr_command_, p2p_traj_spline_duration_);
        time_ = 0;

        p2p_traj_mutex_.unlock();

        return true; 
    }

    bool JointSpaceSplineController::get_cmd_traj_spline_srv(wam_dmp_controller::JointPosSpline::Request &req,
                                                             wam_dmp_controller::JointPosSpline::Response &res)
    {
        // get translation
        res.command.p2p_traj_duration = p2p_traj_spline_duration_;
        // get elapsed time
        res.command.elapsed_time = time_;
        for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
        {
            res.command.joint_pos[i] = prev_setpoint_[i];
        }
        

        return true;        
    }
}
PLUGINLIB_EXPORT_CLASS(wam_dmp_controller::JointSpaceSplineController, controller_interface::ControllerBase)


