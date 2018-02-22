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

#include <boost/scoped_ptr.hpp>

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
		    void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
		    void set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg);
            
            struct Commands
            {
                std::vector<double> positions_;
            };
	    private:

		    ros::Subscriber sub_posture_;
		    ros::Subscriber sub_gains_;
        
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

		    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;

            Commands command_struct_;
            realtime_tools::RealtimeBuffer<Commands> command_buffer_;

	    };
}

#endif
