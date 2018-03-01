/*************************************************************************
	> File Name: joint_space_controller.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 21 Feb 2018 08:34:25 PM PST
 ************************************************************************/

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>
#include <wam_dmp_controller/joint_space_controller.h>

namespace wam_dmp_controller
{
    JointSpaceController::JointSpaceController() {}
	JointSpaceController::~JointSpaceController() {}

	bool JointSpaceController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{
        KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
        
		id_solver_.reset(new KDL::ChainDynParam( kdl_chain_, gravity_));

		cmd_states_.resize(kdl_chain_.getNrOfJoints());
		tau_cmd_.resize(kdl_chain_.getNrOfJoints());
		Kp_.resize(kdl_chain_.getNrOfJoints());
		Kv_.resize(kdl_chain_.getNrOfJoints());
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());
        home_.resize(kdl_chain_.getNrOfJoints());
        joint_initial_states_.resize(kdl_chain_.getNrOfJoints());
        current_cmd_.resize(kdl_chain_.getNrOfJoints());
        command_struct_.positions_.resize(kdl_chain_.getNrOfJoints());
        // Read the Kq and Kv gains from the coniguration file
        ros::NodeHandle Kp_handle(n, "Kp_Gains");
        ros::NodeHandle Kv_handle(n, "Kv_Gains");
        ros::NodeHandle home_handle(n, "home_pos");
        
        for (size_t i = 0; i < joint_handles_.size(); i++)
        {
            std::cout << joint_handles_[i].getName();
            if ( !Kp_handle.getParam(joint_handles_[i].getName(), Kp_(i) ) ) 
            {
                ROS_WARN("Kp gain not set for %s in yaml file, Using 300.", joint_handles_[i].getName().c_str());
                Kp_(i) = 300;
            }
            else 
            {
                ROS_WARN("Kp gain set for %s in yaml file, Using %f", joint_handles_[i].getName().c_str(), Kp_(i)); 
            }

            if ( !Kv_handle.getParam(joint_handles_[i].getName().c_str(), Kv_(i) ) ) 
            {
                ROS_WARN("Kv gain not set for %s in yaml file, Using 0.7.", joint_handles_[i].getName().c_str());
                Kv_(i) = 0.7;
            }
            else 
            {
                 ROS_WARN("Kv gain set for %s in yaml file, Using %f", joint_handles_[i].getName().c_str(), Kv_(i)); 
            }

            if ( !home_handle.getParam(joint_handles_[i].getName().c_str(), home_(i))) 
            {
                ROS_WARN("home pos not set for %s in yaml file, Using 0.0.", joint_handles_[i].getName().c_str());
                home_(i) = 0.0;
            }
            else 
            {
                 ROS_WARN("home pos set for %s in yaml file, Using %f", joint_handles_[i].getName().c_str(), home_(i)); 
            }            
        }
        sub_posture_ = nh_.subscribe("command", 1, &JointSpaceController::command, this);
		set_posture_service_ = nh_.advertiseService("set_joint_pos", &JointSpaceController::set_joint_pos, this);
		set_gains_service_ = nh_.advertiseService("set_gains", &JointSpaceController::set_gains, this);
        get_posture_service_ = nh_.advertiseService("get_joint_pos", &JointSpaceController::get_joint_pos, this);
		get_gains_service_ = nh_.advertiseService("get_gains", &JointSpaceController::get_gains, this);
        go_home_service_ = nh_.advertiseService("go_home", &JointSpaceController::go_home, this);
		return true;		
	}

    void JointSpaceController::starting(const ros::Time& time)
	{
  		// get joint positions
  		for(size_t i=0; i<joint_handles_.size(); i++) 
  		{
            /*
  			Kp_(i) = 300.0;
  			Kv_(i) = 0.7;
            */
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            cmd_states_(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_msr_states_.qdotdot(i) = 0.0;
    		joint_des_states_.q(i) = joint_msr_states_.q(i);
            //command_buffer_.initRT(command_struct_);
    	}
        command_buffer_.initRT(command_struct_);
        
        lambda = 0.1;	// lower values: flatter
    	cmd_flag_ = 0;	
    	step_ = 0;

    	ROS_INFO(" Number of joints in handle = %lu", joint_handles_.size() );
    }

    void JointSpaceController::update(const ros::Time& time, const ros::Duration& period)
    {
    	// get joint positions
  		for(size_t i=0; i<joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_msr_states_.qdotdot(i) = 0.0;
    	}

    	if(cmd_flag_)
    	{
            command_struct_ = *(command_buffer_.readFromRT());
            if(step_ == 0)
            {
                joint_initial_states_ = joint_msr_states_.q;
            }
            // reaching desired joint position using a hyperbolic tangent function
            double th = tanh(M_PI-lambda*step_);
            double ch = cosh(M_PI-lambda*step_);
            double sh2 = 1.0/(ch*ch);
            
            for(size_t i=0; i<joint_handles_.size(); i++)
            {
                // TODO: take into account also initial/final velocity and acceleration
                cmd_states_(i) = command_struct_.positions_[i];
                current_cmd_(i) = cmd_states_(i) - joint_initial_states_(i);
                joint_des_states_.q(i) = current_cmd_(i)*0.5*(1.0-th) + joint_initial_states_(i);
                joint_des_states_.qdot(i) = current_cmd_(i)*0.5*lambda*sh2;
                joint_des_states_.qdotdot(i) = current_cmd_(i)*lambda*lambda*sh2*th;
            }
            ++step_;

    		if(joint_des_states_.q == cmd_states_)
    		{
    			cmd_flag_ = 0;	//reset command flag
                ROS_INFO("Total Step: %u", step_);                
    			step_ = 0;
    			ROS_INFO("Posture OK");
    		}
    	}

    	// computing Inertia, Coriolis and Gravity matrices
    	id_solver_->JntToMass(joint_msr_states_.q, M_);
    	id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
    	id_solver_->JntToGravity(joint_msr_states_.q, G_);

        // PID controller
        KDL::JntArray pid_cmd_(joint_handles_.size());
        // compensation of Coriolis and Gravity
        KDL::JntArray cg_cmd_(joint_handles_.size());
        for(size_t i=0; i<joint_handles_.size(); i++)
        {
            // control law
            pid_cmd_(i) = joint_des_states_.qdotdot(i) + Kv_(i)*(joint_des_states_.qdot(i) - joint_msr_states_.qdot(i)) + Kp_(i)*(joint_des_states_.q(i) - joint_msr_states_.q(i));
            cg_cmd_(i) = C_(i)*joint_msr_states_.qdot(i) + G_(i);
        }
        tau_cmd_.data = M_.data * pid_cmd_.data;
        KDL::Add(tau_cmd_,cg_cmd_,tau_cmd_);
        
        for(size_t i=0; i<joint_handles_.size(); i++)
        {
            joint_handles_[i].setCommand(tau_cmd_(i));
        }
        /*
        if (step_ != 0)
        {
            std::cout << "current joint pos: " <<joint_msr_states_.q(0)<< " " << 	joint_msr_states_.q(1)<< " " << 	joint_msr_states_.q(2)<< " " << 	joint_msr_states_.q(3)<< " " << 	joint_msr_states_.q(4)<< " " << 	joint_msr_states_.q(5) << 	joint_msr_states_.q(6)<< std::endl;
        }
        */
    }

    void JointSpaceController::command(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if(msg->data.size() == 0)
    		ROS_ERROR("Desired configuration must be of dimension %lu", joint_handles_.size());
    	else if(msg->data.size() != joint_handles_.size())
    	{
    		ROS_ERROR("Posture message had the wrong size: %u", (unsigned int)msg->data.size());
    		return;
    	}
    	else
    	{   if (cmd_flag_ != 1)
            {
    		    for (unsigned int i = 0; i<joint_handles_.size(); i++)
    		    {	
                    command_struct_.positions_[i] = msg->data[i];
                }
                command_buffer_.writeFromNonRT(command_struct_);
    		    cmd_flag_ = 1;
                // when a new command is set, steps should be reset to avoid jumps in the update
                step_ = 0;
    	    }
            else 
            {
                ROS_WARN("Current executing previous command, wait!");
            }
	    }
    }

    bool JointSpaceController::set_joint_pos(wam_dmp_controller::SetJointPos::Request &req,
                                             wam_dmp_controller::SetJointPos::Response &res)
    {
        if(req.command.joint_pos.size() == 0)
    	{
            ROS_ERROR("Joint posture array must be of dimension %lu", joint_handles_.size());
            return res.state = false;
        }
    	else if(req.command.joint_pos.size() != joint_handles_.size())
    	{
    		ROS_ERROR("Joint posture array had the wrong size: %u", (unsigned int)req.command.joint_pos.size());
    		return res.state = false;
    	}
    	else
    	{
            if (cmd_flag_ != 1)
            {
                
                for (unsigned int i = 0; i<joint_handles_.size(); i++)
    		    {	
                    command_struct_.positions_[i] = req.command.joint_pos[i];
                }
                command_buffer_.writeFromNonRT(command_struct_);
    		    cmd_flag_ = 1;
                // when a new command is set, steps should be reset to avoid jumps in the update
                step_ = 0;

                res.state = true;
    	    }
            else 
            {
                ROS_WARN("[SetJointPos]:Currently executing previous command, wait!");
                res.state = false;
            }
        }
        return true;
	}

    bool JointSpaceController::get_joint_pos(wam_dmp_controller::GetJointPos::Request &req,
                                             wam_dmp_controller::GetJointPos::Response &res)
    {
        res.command.joint_pos.resize(joint_handles_.size());
        for (int i = 0; i < joint_handles_.size(); i++)
        {
            res.command.joint_pos[i] = cmd_states_(i);
            ROS_INFO("JOINT GOAL: %f", res.command.joint_pos[i]);
        }
        res.state = true;
        return res.state;
    }
    
    bool JointSpaceController::go_home(wam_dmp_controller::GoHome::Request &req,
                                       wam_dmp_controller::GoHome::Response &res)
    {
        if (cmd_flag_ != 1)
        {
            for (unsigned int i = 0; i<joint_handles_.size(); i++)
    	    {	
                command_struct_.positions_[i] = home_(i);
            }
            command_buffer_.writeFromNonRT(command_struct_);
    	    cmd_flag_ = 1;
            // when a new command is set, steps should be reset to avoid jumps in the update
            step_ = 0;
        
            res.state = true;
        }
        else 
        {
            ROS_WARN("[GoHome]:Currently executing previous command, wait!");
        }
        return true;
    }

    bool JointSpaceController::set_gains(wam_dmp_controller::SetJointGains::Request &req,
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

    bool JointSpaceController::get_gains(wam_dmp_controller::GetJointGains::Request &req,
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
PLUGINLIB_EXPORT_CLASS(wam_dmp_controller::JointSpaceController, controller_interface::ControllerBase)

