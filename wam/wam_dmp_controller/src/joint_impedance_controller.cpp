/*************************************************************************
	> File Name: joint_impedance_controller.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 21 Feb 2018 04:27:23 PM PST
 ************************************************************************/


#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <wam_dmp_controller/joint_impedance_controller.h>

namespace wam_dmp_controller
{
    JointImpedanceController::JointImpedanceController() {}

    JointImpedanceController::~JointImpedanceController() {}

    bool JointImpedanceController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
        KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
        Kp_.resize(kdl_chain_.getNrOfJoints());
        Kd_.resize(kdl_chain_.getNrOfJoints());   
        q_des_.resize(kdl_chain_.getNrOfJoints());
        tau_des_.resize(kdl_chain_.getNrOfJoints());
        //command_vec.resize(kdl_chain_.getNrOfJoints());
        //command_buffer_vec.resize(kdl_chain_.getNrOfJoints());
 
        for (size_t i = 0; i < joint_handles_.size(); i++)
        {
            //command_vec.push_back(Commands());
            //command_buffer_vec.push_back(realtime_tools::RealtimeBuffer<Commands>());
            tau_des_(i) = joint_handles_[i].getPosition();
            Kp_(i) = joint_stiffness_handles_[i].getPosition();
            Kd_(i) = joint_damping_handles_[i].getPosition();
            q_des_(i) = joint_set_point_handles_[i].getPosition();
        }

        ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );

        for (int i = 0; i < joint_handles_.size(); ++i) {
            if ( !nh_.getParam("stiffness_gains", Kp_(i) ) ) {
                ROS_WARN("Stiffness gain not set in yaml file, Using %f", Kp_(i));
            }
        }
        for (int i = 0; i < joint_handles_.size(); ++i) {
            if ( !nh_.getParam("damping_gains", Kd_(i)) ) {
                ROS_WARN("Damping gain not set in yaml file, Using %f", Kd_(i));
            }
        }

        typedef  const std_msgs::Float64MultiArray::ConstPtr& msg_type;
        sub_stiffness_ = nh_.subscribe<JointImpedanceController, msg_type>("stiffness", 1, boost::bind(&JointImpedanceController::setParam, this, _1, &Kp_, "Kp"));
        sub_damping_ = nh_.subscribe<JointImpedanceController, msg_type>("damping", 1, boost::bind(&JointImpedanceController::setParam, this, _1, &Kd_, "Kd"));
        sub_add_torque_ = nh_.subscribe<JointImpedanceController, msg_type>("additional_torque", 1, boost::bind(&JointImpedanceController::setParam, this, _1, &tau_des_, "AddTorque"));
        sub_posture_ = nh_.subscribe("command", 1, &JointImpedanceController::command, this);

        return true;
    }

    void JointImpedanceController::starting(const ros::Time& time)
    {
        // Initializing stiffness, damping, ext_torque and set point values
        for (size_t i = 0; i < joint_handles_.size(); i++) 
        {
            tau_des_(i) = 0.0;
            q_des_(i) = joint_handles_[i].getPosition();
            //command_vec[i].position_ = q_des_(i);
            //command_buffer_.initRT(command_struct_);
        }
        command_buffer_.initRT(command_struct_);
    }
    

    void JointImpedanceController::update(const ros::Time& time, const ros::Duration& period)
    {
        command_struct_ = *(command_buffer_.readFromRT());
        //Compute control law. This controller sets all variables for the JointImpedance Interface from kuka
        for (size_t i = 0; i < joint_handles_.size(); i++)
        {
            double joint_q_msr = joint_handles_[i].getPosition();
            double joint_dq_msr = joint_handles_[i].getVelocity();
            q_des_(i) = command_struct_.positions_[i];
            tau_des_(i) = Kp_(i) * (joint_q_msr - q_des_(i)) + Kd_(i) * joint_dq_msr; 
            joint_handles_[i].setCommand(tau_des_(i));
            joint_stiffness_handles_[i].setCommand(Kp_(i));
            joint_damping_handles_[i].setCommand(Kd_(i));
            joint_set_point_handles_[i].setCommand(q_des_(i));
        }

    }

    void JointImpedanceController::command(const std_msgs::Float64MultiArray::ConstPtr &msg) 
    {
        if (msg->data.size() == 0) {
            ROS_INFO("Desired configuration must be: %lu dimension", joint_handles_.size());
        }
        else if ((int)msg->data.size() != joint_handles_.size()) {
            ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
            return;
        }
        else
        {
            for (unsigned int j = 0; j < joint_handles_.size(); ++j)
            {    //q_des_(j) = msg->data[j];
                 command_struct_.positions_[j] = msg->data[j];
                //command_buffer_vec[j].writeFromNonRT(command_vec[j]);
            }
            command_buffer_.writeFromNonRT(command_struct_);
        }
    }

    void JointImpedanceController::setParam(const std_msgs::Float64MultiArray_< std::allocator< void > >::ConstPtr& msg, KDL::JntArray* array, std::string s)
    {
        if (msg->data.size() == joint_handles_.size())
        {
            for (unsigned int i = 0; i < joint_handles_.size(); ++i)
            {
                (*array)(i) = msg->data[i];
            }
        }
        else
        {
            ROS_INFO("Num of Joint handles = %lu", joint_handles_.size());
        }

        ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

        ROS_INFO("New param %s: %.2lf, %.2lf, %.2lf %.2lf, %.2lf, %.2lf, %.2lf", s.c_str(),
             (*array)(0), (*array)(1), (*array)(2), (*array)(3), (*array)(4), (*array)(5), (*array)(6));
    }

}
PLUGINLIB_EXPORT_CLASS(wam_dmp_controller::JointImpedanceController, controller_interface::ControllerBase)

