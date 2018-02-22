/*************************************************************************
	> File Name: gravity_compensation.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 22 Feb 2018 10:49:12 AM PST
 ************************************************************************/

#ifndef __WAM_DMP_CONTROLLER_GRAVITY_COMPENSATION_H
#define __WAM_DMP_CONTROLLER_GRAVITY_COMPENSATION_H

#include "KinematicChainControllerBase.h"
#include <boost/scoped_ptr.hpp>

namespace wam_dmp_controller
{
    class GravityCompensation: public wam_dmp_controller::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
    {
    public:
        
        GravityCompensation();
        ~GravityCompensation();
        
        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& time);
        
    private:
        KDL::JntArray G_;
        boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
        
        // std::vector<float> previous_stiffness_; /// stiffness before activating controller
        
        // hack required as long as there is separate position handle for stiffness
        // std::vector<hardware_interface::JointHandle> joint_stiffness_handles_;
        
        // const static float DEFAULT_STIFFNESS = 0.01;
        
    };
}

#endif
