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

        set_cmd_traj_pos_service_ = n.advertiseService("set_traj_pos_spline",
                                                       &JointSpaceSplineController::set_cmd_traj_spline_srv, this);
        get_cmd_traj_pos_service_ = n.advertiseService("get_traj_pos_spline",
                                                       &JointSpaceSplineController::get_cmd_traj_spline_srv, this);
        go_home_traj_service_ =  n.advertiseService("go_home_traj_spline",
                                                    &JointSpaceSplineController::go_home_traj_spline_srv, this);
        set_gains_service_ = nh_.advertiseService("set_gains", &JointSpaceSplineController::set_gains, this);
        get_gains_service_ = nh_.advertiseService("get_gains", &JointSpaceSplineController::get_gains, this);

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

        prev_setpoint_state_.reset(new PosVelAccState<double>(kdl_chain_.getNrOfJoints()));
        curr_setpoint_state_.reset(new PosVelAccState<double>(kdl_chain_.getNrOfJoints()));
        //prev_setpoint_state_ = PosVelAccState<double>(kdl_chain_.getNrOfJoints());
        //curr_setpoint_state_ = PosVelAccState<double>(kdl_chain_.getNrOfJoints());


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
        {
            q(i) = joint_handles_[i].getPosition();
            
            prev_setpoint_state_->position[i]     = q(i);
            prev_setpoint_state_->velocity[i]     = 0.0;
            prev_setpoint_state_->acceleration[i] = 0.0;
            curr_setpoint_state_->position[i]     = q(i);
            curr_setpoint_state_->velocity[i]     = 0.0;
            curr_setpoint_state_->acceleration[i] = 0.0;
            
            /*
            prev_setpoint_state_.position[i]     = q(i);
            prev_setpoint_state_.velocity[i]     = 0.0;
            prev_setpoint_state_.acceleration[i] = 0.0;
            curr_setpoint_state_.position[i]     = q(i);
            curr_setpoint_state_.velocity[i]     = 0.0;
            curr_setpoint_state_.acceleration[i] = 0.0;
            */
            
        }
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
        spline_seg_.reset(new QuinticSplineSegment<double>(0, *prev_setpoint_state_, p2p_traj_spline_duration_, *curr_setpoint_state_));
        time_ = p2p_traj_spline_duration_;
        
        // Let's use quinc spline segment 

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

        //use the spline segment here 
        PosVelAccState<double> des_state; 
        spline_seg_->sample(time_, des_state);
        /* 
        for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
        {
            q_des[i] = des_state.position[i];
            q_dot_des[i] = des_state.velocity[i];
            q_dotdot_des_[i] = des_state.acceleration[i];
        }
        */
    

        for (int i=0; i<kdl_chain_.getNrOfJoints(); i++)
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
            /* use the spling segment
            curr_setpoint_state_->position[i] = command_struct_.positions_[i];
            prev_setpoint_state_->position[i] = curr_setpoint_state_->position[i];
            prev_setpoint_state_->velocity[i] = curr_setpoint_state_->velocity[i];
            prev_setpoint_state_->acceleration[i] = curr_setpoint_state_->acceleration[i];
            */
        }
        curr_command_ = Eigen::Map<Eigen::VectorXd>(&command_struct_.positions_[0], command_struct_.positions_.size());
        p2p_traj_mutex_.lock();

        p2p_traj_spline_duration_ = req.command.p2p_traj_duration;

        //command_buffer_.writeFromNonRT(command_struct_); Don't do this when set a spline!!!!!! 
        eval_point_to_point_traj_constants(curr_command_, p2p_traj_spline_duration_);
        //curr_setpoint_state_->velocity = std::vector<double>(kdl_chain_.getNrOfJoints(), 0.0);
        //curr_setpoint_state_->acceleration = std::vector<double>(kdl_chain_.getNrOfJoints(), 0.0);
        // when use the segspline class just refit the spline segment 
        //  spline_seg_.reset(new QuinticSplineSegment<double>(0, *prev_setpoint_state_, p2p_traj_spline_duration_, *curr_setpoint_state_));

        time_ = 0;

        p2p_traj_mutex_.unlock();

        return true; 
    }

     bool JointSpaceSplineController::go_home_traj_spline_srv(wam_dmp_controller::GoHomeSpline::Request &req, 
                                                              wam_dmp_controller::GoHomeSpline::Response &res)
    {
        if (command_struct_.positions_.size() != kdl_chain_.getNrOfJoints())
        {
            ROS_ERROR("command struct has size %lu, home posture has size %u, they are not equal", 
                      command_struct_.positions_.size(), 
                      kdl_chain_.getNrOfJoints());
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
            command_struct_.positions_[i] = home_(i);
            /*
            curr_setpoint_state_->position[i] = home_(i);
            prev_setpoint_state_->position[i] = curr_setpoint_state_->position[i];
            prev_setpoint_state_->velocity[i] = curr_setpoint_state_->velocity[i];
            prev_setpoint_state_->acceleration[i] = curr_setpoint_state_->acceleration[i];
            */
        }
        //curr_setpoint_state_->velocity = std::vector<double>(kdl_chain_.getNrOfJoints(), 0.0);
        //curr_setpoint_state_->acceleration = std::vector<double>(kdl_chain_.getNrOfJoints(), 0.0);
        curr_command_ = Eigen::Map<Eigen::VectorXd>(&command_struct_.positions_[0], command_struct_.positions_.size());
        p2p_traj_mutex_.lock();

        p2p_traj_spline_duration_ = DEFAULT_P2P_TRAJ_DURATION;

        //command_buffer_.writeFromNonRT(command_struct_); Don't do this when set a spline!!!!!! 
        eval_point_to_point_traj_constants(curr_command_, p2p_traj_spline_duration_);
        // when use the segspline class just refit the spline segment 
        //  spline_seg_.reset(new QuinticSplineSegment<double>(0, *prev_setpoint_state_, p2p_traj_spline_duration_, *curr_setpoint_state_));
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
            // res.command.joint_pos[i] = prev_setpoint_state_->position[i];
        }
        

        return true;        
    }

    bool JointSpaceSplineController::set_gains(wam_dmp_controller::SetJointGains::Request &req,
                                         wam_dmp_controller::SetJointGains::Response &res)
	{
        if(req.command.Kp_gains.size() == 0)
    	{
            ROS_ERROR("Desired Kp array must be of dimension %lu", joint_handles_.size());
            return res.state = false;
        }
    	else if(req.command.Kp_gains.size() != joint_handles_.size())
    	{
    		ROS_ERROR("Kp array had the wrong size: %u", (unsigned int)req.command.Kp_gains.size());
    		return res.state = false;
    	}

        if(req.command.Kv_gains.size() == 0)
    	{
            ROS_ERROR("Desired Kv array must be of dimension %lu", joint_handles_.size());
            return res.state = false;
        }
    	else if(req.command.Kp_gains.size() != joint_handles_.size())
    	{
    		ROS_ERROR("Kv array had the wrong size: %u", (unsigned int)req.command.Kv_gains.size());
    		return res.state = false;
    	}

        if (time_ != 0)
        {
            ROS_WARN("Set the gains after the current trajectory is done!!!!!!");
            res.state = true;
            return res.state;
        }
		
		for(unsigned int i = 0; i < joint_handles_.size(); i++)
		{
				Kp_(i) = req.command.Kp_gains[i];
				Kv_(i) = req.command.Kv_gains[i];
		}
		
		ROS_INFO("New gains Kp: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kp_(0), Kp_(1), Kp_(2), Kp_(3), Kp_(4), Kp_(5), Kp_(6));
		ROS_INFO("New gains Kv: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kv_(0), Kv_(1), Kv_(2), Kv_(3), Kv_(4), Kv_(5), Kv_(6));
        res.state = true;
        return res.state;
	}

    bool JointSpaceSplineController::get_gains(wam_dmp_controller::GetJointGains::Request &req,
                                               wam_dmp_controller::GetJointGains::Response &res)
    {
        res.command.Kp_gains.resize(joint_handles_.size());
        res.command.Kv_gains.resize(joint_handles_.size());
        for (int i = 0; i < joint_handles_.size(); i++)
        {
            res.command.Kp_gains[i] = Kp_(i);
            res.command.Kv_gains[i] = Kv_(i);
            ROS_INFO("Gains Kp, Kv:%f, %f", res.command.Kp_gains[i], res.command.Kv_gains[i]);
        }
        res.state = true;
        return res.state;
    }

}
PLUGINLIB_EXPORT_CLASS(wam_dmp_controller::JointSpaceSplineController, controller_interface::ControllerBase)


