/*************************************************************************
	> File Name: cartesian_twist_controller_ik_with_nullspace_optimization.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 09 Jun 2017 04:57:51 PM PDT
 ************************************************************************/

#include <iostream>
#include <algorithm>
#include <sstream>

// ros includes 
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/kinfam_io.hpp>

#include <angles/angles.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes 
#include <wam_dmp_controller/cartesian_twist_controller_ik_with_nullspace_optimization.h>

// import most comman eigen types 
using namespace Eigen;
using namespace std;

namespace wam_dmp_controller
{
    static const int NUM_JOINTS = 7;
    static const int NUM_CART = 6;

    CartesianTwistControllerWithNullspaceOptimization::CartesianTwistControllerWithNullspaceOptimization() : jnt_to_twist_solver_(NULL), jnt_to_pose_solver_(NULL), jnt_to_jac_solver_(NULL), num_joints_(0), publisher_counter_(0), publisher_buffer_size_(0), header_sequence_number_(0)
    {
    }
    
    bool CartesianTwistControllerWithNullspaceOptimization::readParameters()
    {
        ROS_VERIFY(usc_utilities::read(node_handle_, std::string("damping"), damping_));
        ROS_VERIFY(usc_utilities::read(node_handle_, std::string("publisher_buffer_size"), publisher_buffer_size_));
        ROS_VERIFY(usc_utilities::read(node_handle_, std::string("publisher_rate"), publisher_rate_));

        //ros::NodeHandle cartesian_ff_gains_handle(std::string("/cartesian_pose_twist_gains"));
        ROS_VERIFY(usc_utilities::read(node_handle_, std::string("ff_trans"), ff_trans_));
        ROS_VERIFY(usc_utilities::read(node_handle_, std::string("ff_rot"), ff_rot_));
        return true;
    }

    bool CartesianTwistControllerWithNullspaceOptimization::initMechanismChain()
    {
        // get names of root and tip joint from the parameter server 
        std::string root_name, tip_name, urdf_str;
        
        if (!node_handle_.getParam("root_name", root_name))
        {
            ROS_ERROR("Could not retrieve parameter >>root<< from the parameter server in the namespace %s.", node_handle_.getNamespace().c_str());
            return false;
        }

        if (!node_handle_.getParam("tip_name", tip_name))
        {
            ROS_ERROR("Could not retrieve parameter >>tip<< from the parameter server in the namespace %s.", node_handle_.getNamespace().c_str());
            return false;
        }
        
        // get the urdf model from the parameter server 
        ros::NodeHandle temp_nh("barrett");
        ROS_VERIFY(usc_utilities::read(temp_nh, "robot_description_yi", urdf_str));
        if (!robot_urdf_.initString(urdf_str))
        {
            ROS_ERROR("Could not load urdf model from the paramter >>robot_description<< from the paramter server in the namespace %s.", node_handle_.getNamespace().c_str());
            return false;
        }

        // Construct the kdl chain 
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(robot_urdf_, kdl_tree))
        {
            ROS_ERROR("Could not convert urdf into kdl tree.");
            return false;
        }

        bool res;
        try 
        {
            res = kdl_tree.getChain(root_name, tip_name, kdl_chain_);
        }
        catch(...)
        {
            ROS_ERROR("Could not extract chain between %s and %s from the kdl tree.", root_name.c_str(), tip_name.c_str());
            return false;
        }

        if (static_cast<int>(kdl_chain_.getNrOfJoints()) != NUM_JOINTS)
        {
            ROS_ERROR("For now, the KDL chain needs to have >%i< arm joints, but only has >%i<", NUM_JOINTS, static_cast<int>(kdl_chain_.getNrOfJoints()));
            return false;
        }

        // Initialize all the KDL solvers and matrix 
        jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        jnt_to_twist_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        
        kdl_chain_jacobian_.resize(NUM_JOINTS);
        kdl_current_joint_positions_.resize(NUM_JOINTS);
        kdl_current_joint_velocities_.resize(NUM_JOINTS);
        kdl_desired_joint_positions_.resize(NUM_JOINTS);

        return true;
    }

    bool CartesianTwistControllerWithNullspaceOptimization::initCartesianPidControllers()
    {
        // Construct 3 identical pid controllers for the x, y and z translations
        //boost::shared_ptr<control_toolbox::Pid> pid_controller(boost::make_shared<control_toolbox::pid>());
        control_toolbox::Pid pid_controller;
        if (!pid_controller.init(ros::NodeHandle(node_handle_, "fb_trans")))
        {
            ROS_ERROR("Could not contruct pid controller for the x,y and z translations!");
            return false;
        }
        for (int i = 0; i < 3; i++)
        {
            cartesian_fb_pid_controllers_.push_back(pid_controller);
        }

        // Construct 3 identitcal pid controllers for the roll, pitch and yaw rotations
        //pid_controller.reset(new control_toolbox::pid());
        if (!pid_controller.init(ros::NodeHandle(node_handle_, "fb_rot")))
        {
            ROS_ERROR("Could not contruct pid controller for the raw, pitch, yaw rotations!");
            return false;
        }
        for (int i = 0; i < 3; i++)
        {
            cartesian_fb_pid_controllers_.push_back(pid_controller);
        }

        return true;
    }

    bool CartesianTwistControllerWithNullspaceOptimization::initNullspacePidControllers()
    {
        std::string null_space_controller_names;
        if (!node_handle_.getParam("nullspace_controller_names", null_space_controller_names))
        {
            ROS_ERROR("Could not retrieve parameter >>null_space_controller_names<< from the parameter server in the namespace %s!", node_handle_.getNamespace().c_str());
            return false;
        }

        // split names based on white space 
        std::stringstream ss_null_space_controller_names(null_space_controller_names);
        std::string null_space_controller_name;
        while (ss_null_space_controller_names >> null_space_controller_name)
        {
            boost::shared_ptr<control_toolbox::Pid> pid_controller(boost::make_shared<control_toolbox::Pid>());                    
            if (!pid_controller->init(ros::NodeHandle(node_handle_, null_space_controller_name)))
            {
                ROS_ERROR("Could not construct pid controller for %s.", null_space_controller_name.c_str());
                return false;
            }
            nullspace_fb_pid_controllers_.push_back(pid_controller);

            // Debug Info 
            ROS_DEBUG_STREAM("Construct pid controller: >>%s<<." <<  null_space_controller_name);
        }

        if (nullspace_fb_pid_controllers_.size() != num_joints_)
        {
            ROS_ERROR("There are %i nullspace terms but there should be %i",(int) nullspace_fb_pid_controllers_.size(), num_joints_);
            return false;
        }

        return true;
    }

    bool CartesianTwistControllerWithNullspaceOptimization::initRTPublisher()
    {
        pose_twist_desired_publisher_.reset(new realtime_tools::RealtimePublisher<wam_dmp_controller::PoseTwistStamped>(node_handle_, "pose_twist_desired_stamped", 10));
        pose_twist_actual_publisher_.reset(new realtime_tools::RealtimePublisher<wam_dmp_controller::PoseTwistStamped>(node_handle_, "pose_twist_actual_stamped", 10));
        nullspace_term_publisher_.reset(new realtime_tools::RealtimePublisher<wam_dmp_controller::NullspaceTermStamped>(node_handle_, "null_space_term_stamped", 10));

        return true;
    }

    bool CartesianTwistControllerWithNullspaceOptimization::initJointPositionController(hardware_interface::EffortJointInterface *hw, ros::NodeHandle node_handle, std::vector<boost::shared_ptr<JointPositionController> >& joint_position_controllers)
    {
        joint_position_controllers.clear();
        std::vector<std::string> joint_names;
        ROS_VERIFY(usc_utilities::read(node_handle, "joint_names", joint_names));
        for (int i = 0; i < (int)joint_names.size(); i++)
        {
            ros::NodeHandle joint_node_handle(node_handle, joint_names[i]);
            boost::shared_ptr<JointPositionController> joint_position_controller(boost::make_shared<JointPositionController>());
            if (!joint_position_controller->init(hw, joint_node_handle))
            {
                ROS_ERROR("Could not initialize controller for joint >%s<.", joint_names[i].c_str());
                return false;
            }
            joint_position_controllers_.push_back(joint_position_controller);
        }

        if (static_cast<int>(joint_position_controllers_.size()) == NUM_JOINTS)
        {
            return true;
        }
        else 
        {
            ROS_ERROR("There should be >>%i<< controllers, instead there are >>%i<< controllers!", NUM_JOINTS, (int)joint_position_controllers_.size());
            return false;
        }
    }
    
    bool CartesianTwistControllerWithNullspaceOptimization::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &node_handle)
    {
        ROS_ASSERT(hw);
        hw_ = hw;
        node_handle_ = node_handle;

        rest_posture_joint_configuration_ = VectorXd::Zero(NUM_JOINTS, 1);
        eigen_desired_cartesian_velocities_ = VectorXd::Zero(NUM_CART, 1);
        eigen_desired_joint_positions_ = VectorXd::Zero(NUM_JOINTS, 1);
        eigen_desired_joint_velocities_ = VectorXd::Zero(NUM_JOINTS, 1);

        eigen_chain_jacobian_ = MatrixXd::Zero(NUM_CART, NUM_JOINTS);

        eigen_jac_times_jac_transpose_ = MatrixXd::Zero(NUM_CART, NUM_CART);
        eigen_jjt_inverse_ = MatrixXd::Zero(NUM_CART, NUM_CART);
        eigen_jac_pseudo_inverse_ = MatrixXd::Zero(NUM_CART, NUM_JOINTS);
        eigen_identity_ = MatrixXd::Zero(NUM_JOINTS, NUM_JOINTS);
        eigen_identity_.setIdentity(NUM_JOINTS, NUM_JOINTS);
        

        eigen_nullspace_projector_ = MatrixXd::Zero(NUM_JOINTS, NUM_JOINTS);
        eigen_nullspace_term_ = MatrixXd::Zero(NUM_JOINTS, 1);
        eigen_nullspace_error_ = MatrixXd::Zero(NUM_JOINTS, 1);

        ROS_VERIFY(readParameters());
        ROS_VERIFY(initMechanismChain());
        ROS_VERIFY(initJointPositionController(hw, node_handle, joint_position_controllers_));

        num_joints_ = static_cast<int>(joint_position_controllers_.size());
        ROS_ASSERT(num_joints_ == NUM_JOINTS);

        ROS_VERIFY(initCartesianPidControllers());
        ROS_VERIFY(initNullspacePidControllers());
        pose_unfiltered_data_.resize(NUM_CART);
        pose_filtered_data_.resize(NUM_CART);

        if (!((filters::MultiChannelFilterBase<double>&)pose_filter_).configure(NUM_CART, node_handle_.getNamespace() + std::string("/pose_filter"), node_handle_))
        {
            ROS_ERROR("Could not read velocity filter!");
            return false;
        }

        ROS_VERIFY(initRTPublisher());
        
        return true;
    }

    void CartesianTwistControllerWithNullspaceOptimization::starting(const ros::Time& time)
    {
        // reset cartesian space and null space pid controllers 
        for (int i = 0; i < NUM_CART; i++)
        {
            cartesian_fb_pid_controllers_[i].reset();
        }

        for (int i = 0; i < num_joints_; i++)
        {
            nullspace_fb_pid_controllers_[i]->reset();
        }

        // start joint position controller ****************
        for (int i = 0; i < num_joints_; i++)
        {
            joint_position_controllers_[i]->starting(time);
        }
        
        // Set the measured Twist to zero
        kdl_twist_measured_ = KDL::Twist::Zero();

        // Get the current joint positions 
        ROS_ASSERT(kdl_current_joint_positions_.rows() == joint_position_controllers_.size());
        for (unsigned int i = 0; i < joint_position_controllers_.size(); i++)
        {
            kdl_current_joint_positions_(i) = joint_position_controllers_[i]->getJointPosition();

            // Normalize the angles if the joint type is Continuous 
            if (joint_position_controllers_[i]->getJointType() == urdf::Joint::CONTINUOUS)
            {
                ROS_DEBUG("Starting: The type for joint >>%i<< is Continuous!", (int)i);
                kdl_current_joint_positions_(i) = angles::normalize_angle(kdl_current_joint_positions_(i));
            }

            // Set the desired output to the current values 
            kdl_desired_joint_positions_(i) = kdl_current_joint_positions_(i);
            eigen_desired_joint_positions_(i) = kdl_current_joint_positions_(i);

            // Set the desired rest posture to current 
            rest_posture_joint_configuration_(i) = kdl_current_joint_positions_(i);
            //ROS_INFO("joint position %i: %f", (int)i, kdl_current_joint_positions_(i));
        }

        // Set the desired cartesian pose to current value 
        jnt_to_pose_solver_->JntToCart(kdl_current_joint_positions_, kdl_pose_desired_);

        last_time_ = time;
    }

    void CartesianTwistControllerWithNullspaceOptimization::update(const ros::Time& time, const ros::Duration& period)
    {
        // get the current joint positions and normalize them 
        for (unsigned int i = 0; i < joint_position_controllers_.size(); i++)
        {
            kdl_current_joint_positions_(i) = joint_position_controllers_[i]->getJointPosition();

            // Normalize the angles if the joint type is Continuous 
            if (joint_position_controllers_[i]->getJointType() == urdf::Joint::CONTINUOUS)
            {
                ROS_DEBUG("Updating: The type for joint >>%i<< is Continuous!", (int)i);
                kdl_current_joint_positions_(i) = angles::normalize_angle(kdl_current_joint_positions_(i));
                eigen_desired_joint_positions_(i) = angles::normalize_angle(eigen_desired_joint_positions_(i));
            }

            kdl_desired_joint_positions_(i) = eigen_desired_joint_positions_(i);
        }

        // get the chain jacobian
        // TODO why use desired positions rather than current positions
        jnt_to_jac_solver_->JntToJac(kdl_desired_joint_positions_, kdl_chain_jacobian_);

        // Converto to Eigen type for inversion and other math later 
        eigen_chain_jacobian_ = kdl_chain_jacobian_.data; // NUM_CART x NUM_JOINTS
        ROS_VERIFY(eigen_chain_jacobian_.rows() == NUM_CART);
        ROS_VERIFY(eigen_chain_jacobian_.cols() == NUM_JOINTS);

        // compute the pseudo inverse 
        eigen_jac_times_jac_transpose_ = eigen_chain_jacobian_ * eigen_chain_jacobian_.transpose() + MatrixXd::Identity(NUM_CART, NUM_CART) * damping_; // NUM_CART x NUM_CART
        //bool invertible;
        //eigen_jac_times_jac_transpose_.computeInverseWithCheck(eigen_jjt_inverse_, invertible); //NUM_CART x NUM_CART
        eigen_jjt_inverse_ = eigen_jac_times_jac_transpose_.inverse();
        //ROS_ASSERT(invertible);
        eigen_jac_pseudo_inverse_ = eigen_chain_jacobian_.transpose() * eigen_jjt_inverse_; // NUM_JOINTS x NUM_CART

        // compute the nullspace projector 
        eigen_identity_.setIdentity(); // NUM_JOINTS x NUM_JOINTS
        eigen_nullspace_projector_ = eigen_identity_ - (eigen_jac_pseudo_inverse_ * eigen_chain_jacobian_);

        // get the current cartesian pose 
        jnt_to_pose_solver_->JntToCart(kdl_current_joint_positions_, kdl_real_pose_measured_);
        jnt_to_pose_solver_->JntToCart(kdl_desired_joint_positions_, kdl_pose_measured_);

        // compute twist 
        kdl_twist_error_ = -diff(kdl_pose_measured_, kdl_pose_desired_);

        // filter twist error 
        for (int i = 0; i < NUM_CART; i++)
        {
            pose_unfiltered_data_[i] = kdl_twist_error_(i);
        }
        pose_filter_.update(pose_unfiltered_data_, pose_filtered_data_);
        for (int i = 0; i < NUM_CART; i++)
        {
            kdl_twist_error_(i) = pose_filtered_data_[i];
        }

        // compute desired cartesian velocities 
        for (int i = 0; i < 3; i++)
        {
            eigen_desired_cartesian_velocities_(i) = (kdl_twist_desired_(i) * ff_trans_) + cartesian_fb_pid_controllers_[i].updatePid(kdl_twist_error_(i), period);
        }
        for (int i = 3; i < NUM_CART; i++)
        {
            eigen_desired_cartesian_velocities_(i) = (kdl_twist_desired_(i) * ff_rot_) + cartesian_fb_pid_controllers_[i].updatePid(kdl_twist_error_(i), period);
        }

        // compute the desired joint velocities
        eigen_desired_joint_velocities_ = eigen_jac_pseudo_inverse_ * eigen_desired_cartesian_velocities_; //NUM_JOINTS X 1

        double error;
        for (int i = 0; i < num_joints_; i++)
        {
            if (joint_position_controllers_[i]->getJointType() == urdf::Joint::CONTINUOUS)
            {
                error = angles::shortest_angular_distance(kdl_current_joint_positions_(i), rest_posture_joint_configuration_(i));
            }
            else 
            {
                error = rest_posture_joint_configuration_(i) - kdl_current_joint_positions_(i);
            }
            eigen_nullspace_error_(i) = nullspace_fb_pid_controllers_[i]->updatePid(error, period);
        }
        eigen_nullspace_term_ = eigen_nullspace_projector_ * eigen_nullspace_error_;
        eigen_desired_joint_velocities_ += eigen_nullspace_term_;

        // integrate desired joint velocities to get desired joint positions 
        eigen_desired_joint_positions_ = eigen_desired_joint_positions_ + eigen_desired_joint_velocities_ * period.toSec();

        //TODO need to clip the joints if the joint type is CONTINUOUS
        // set joint positions and update 
        for (int i = 0; i < num_joints_; i++)
        {
            joint_position_controllers_[i]->setCommand(eigen_desired_joint_positions_(i), eigen_desired_joint_velocities_(i)); 
        }
        for (int i = 0; i < num_joints_; i++)
        {
            joint_position_controllers_[i]->update(time, period); // Joint limits are enforced here 
        }

        // Get measured twist for debugging reasons ... 
        KDL::FrameVel framevel_measured;
        ROS_ASSERT(kdl_current_joint_velocities_.q.rows() == num_joints_);
        ROS_ASSERT(kdl_current_joint_velocities_.qdot.rows() == num_joints_);
        for (int i = 0; i < num_joints_; i++)
        {
            kdl_current_joint_velocities_.q(i) = joint_position_controllers_[i]->getJointPosition();
            kdl_current_joint_velocities_.qdot(i) = joint_position_controllers_[i]->getJointVelocity();
        }
        jnt_to_twist_solver_->JntToCart(kdl_current_joint_velocities_, framevel_measured);
        kdl_twist_measured_ = framevel_measured.deriv();

        publish();
    }

    void CartesianTwistControllerWithNullspaceOptimization::publish()
    {
        publisher_counter_++;
        if (publisher_counter_ & publisher_rate_ == 0)
        {
            header_sequence_number_++;
            
            double qx, qy, qz, qw;
            if (pose_twist_desired_publisher_ && pose_twist_desired_publisher_->trylock())
            {
                pose_twist_desired_publisher_->msg_.header.stamp = ros::Time::now();

                pose_twist_desired_publisher_->msg_.pose.position.x = kdl_pose_desired_.p.x();
                pose_twist_desired_publisher_->msg_.pose.position.y = kdl_pose_desired_.p.y();
                pose_twist_desired_publisher_->msg_.pose.position.z = kdl_pose_desired_.p.z();

                kdl_pose_desired_.M.GetQuaternion(qx, qy, qz, qw);
                pose_twist_desired_publisher_->msg_.pose.orientation.x = qx;
                pose_twist_desired_publisher_->msg_.pose.orientation.y = qy;
                pose_twist_desired_publisher_->msg_.pose.orientation.z = qz;
                pose_twist_desired_publisher_->msg_.pose.orientation.w = qw;

                pose_twist_desired_publisher_->msg_.twist.linear.x = kdl_twist_desired_.vel.x();
                pose_twist_desired_publisher_->msg_.twist.linear.y = kdl_twist_desired_.vel.y();
                pose_twist_desired_publisher_->msg_.twist.linear.z = kdl_twist_desired_.vel.z();
                pose_twist_desired_publisher_->msg_.twist.angular.x = kdl_twist_desired_.rot.x();
                pose_twist_desired_publisher_->msg_.twist.angular.y = kdl_twist_desired_.rot.y();
                pose_twist_desired_publisher_->msg_.twist.angular.z = kdl_twist_desired_.rot.z();

                pose_twist_desired_publisher_->unlockAndPublish();
            }

            if (nullspace_term_publisher_ && nullspace_term_publisher_->trylock())
            {
                nullspace_term_publisher_->msg_.nullspace_term_0 = eigen_nullspace_error_(0);
                nullspace_term_publisher_->msg_.nullspace_term_1 = eigen_nullspace_error_(1);
                nullspace_term_publisher_->msg_.nullspace_term_2 = eigen_nullspace_error_(2);
                nullspace_term_publisher_->msg_.nullspace_term_3 = eigen_nullspace_error_(3);
                nullspace_term_publisher_->msg_.nullspace_term_4 = eigen_nullspace_error_(4);
                nullspace_term_publisher_->msg_.nullspace_term_5 = eigen_nullspace_error_(5);
                nullspace_term_publisher_->msg_.nullspace_term_6 = eigen_nullspace_error_(6);

                nullspace_term_publisher_->unlockAndPublish();
            }
        }
    }
}

