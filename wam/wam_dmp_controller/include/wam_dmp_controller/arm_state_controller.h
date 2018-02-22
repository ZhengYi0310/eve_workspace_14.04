/*************************************************************************
	> File Name: arm_state_controller.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 22 Feb 2018 12:26:34 PM PST
 ************************************************************************/

#ifndef __WAM_DMP_CONTROLLER_ARM_STATE_CONTROLLER_H
#define __WAM_DMP_CONTROLLER_ARM_STATE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <wam_dmp_controller/ArmState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <realtime_tools/realtime_publisher.h>

#include <boost/scoped_ptr.hpp>
namespace wam_dmp_controller
{
    class ArmStateController : public wam_dmp_controller::KinematicChainControllerBase<hardware_interface::JointStateInterface>
    {
    public:
        
        ArmStateController();
        ~ArmStateController();
        
        bool init(hardware_interface::JointStateInterface *robot, ros::NodeHandle &n);
        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& time);
        
    private:
                
        boost::shared_ptr<realtime_tools::RealtimePublisher<wam_dmp_controller::ArmState> > realtime_pub_;
        
        //boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverPos> fk_solver_;
        boost::scoped_ptr<KDL::Jacobian> jacobian_;
        //boost::scoped_ptr<KDL::Vector> gravity_;
        boost::scoped_ptr<KDL::JntArray> joint_position_;
        boost::scoped_ptr<KDL::JntArray> joint_velocity_;
        boost::scoped_ptr<KDL::JntArray> Cartesian_velocity_;
        boost::scoped_ptr<KDL::JntArray> joint_acceleration_;
        //boost::scoped_ptr<KDL::Wrenches> joint_wrenches_;
        //boost::scoped_ptr<KDL::JntArray> joint_effort_est_;
        ros::Time last_publish_time_;
        double publish_rate_;
    };
}
#endif
