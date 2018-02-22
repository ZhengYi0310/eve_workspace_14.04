/*************************************************************************
	> File Name: gravity_compensation.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 22 Feb 2018 10:52:01 AM PST
 ************************************************************************/

#include <pluginlib/class_list_macros.h>
#include <math.h>

#include <wam_dmp_controller/gravity_compensation.h>

namespace wam_dmp_controller 
{
    GravityCompensation::GravityCompensation() {}
    GravityCompensation::~GravityCompensation() {}
    
    bool GravityCompensation::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
        KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
        
        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
        G_.resize(kdl_chain_.getNrOfJoints());
        return true;
    }
    
    void GravityCompensation::starting(const ros::Time& time)
    {
        // for(size_t i=0; i<joint_handles_.size(); i++)
        // {
        //    previous_stiffness_[i] = joint_stiffness_handles_[i].getPosition();
        // }
  		for(size_t i=0; i<joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_msr_states_.qdotdot(i) = 0.0;
            //command_buffer_.initRT(command_struct_);
    	}        
    }
    
    void GravityCompensation::update(const ros::Time& time, const ros::Duration& period)
    {
        // update the commanded position to the actual, so that the robot doesn't 
        // go back at full speed to the last commanded position when the stiffness 
        // is raised again
        id_solver_->JntToGravity(joint_msr_states_.q, G_);
        for(size_t i=0; i<joint_handles_.size(); i++) 
        {
            //joint_handles_[i].setCommand(joint_handles_[i].getPosition());
            joint_handles_[i].setCommand(G_(i));
            // joint_stiffness_handles_[i].setCommand(DEFAULT_STIFFNESS);
        }
    }
    
    void GravityCompensation::stopping(const ros::Time& time)
    {
        //for(size_t i=0; i<joint_handles_.size(); i++)
        //{
        //    joint_stiffness_handles_[i].setCommand(previous_stiffness_[i]);
        //}
    }

}

PLUGINLIB_EXPORT_CLASS(wam_dmp_controller::GravityCompensation, controller_interface::ControllerBase)


