/*************************************************************************
	> File Name: dmp_ik_controller.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 13 Jun 2017 09:00:35 PM PDT
 ************************************************************************/
// system includes 
#include <iostream>
#include <sstream>
#include <boost/thread.hpp>
// ros includes 
#include <ros/callback_queue.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/constants.h>

#include <robot_info/robot_info.h>

//local includes 
#include <wam_dmp_controller/dmp_ik_controller.h>
#include <wam_dmp_controller/dmp_controller.h>
#include <wam_dmp_controller/dmp_controller_implementation.h>

// import most common Eigen types 
using namespace Eigen;

namespace wam_dmp_controller
{
    DMPIKController::DMPIKController() : initialized_(false), publishing_rate_(10), publishing_counter_(0), publisher_buffer_size_(0), visualization_line_counter_(0), visualization_line_rate_(10), visualization_line_max_points_(20), publishing_seq_counter_(0), keep_restposture_fixed_for_testing_(false), last_frame_set_(false), num_joints_(0)
    {
        robot_info::RobotInfo::initialize();
    }

    bool DMPIKController::getArmRelatedVariables(const std::string& handle_namespace, std::string& controller_handle_namespace)
    {
 		/*
        if (handle_namespace.compare(0, 6, std::string("/l_arm")) == 0)
        {
            controller_handle_namespace.assign("barrett/l_arm_dmp_ik_controller");
        }
        else if (handle_namespace.compare(0, 6, std::string("/r_arm")) == 0)
        {
            controller_handle_namespace.assign("barrett/r_arm_dmp_ik_controller");
        }
        else 
        {
            ROS_ERROR("Invalid controller handle namespace: >%s<!", handle_namespace.c_str());
            return false;
        }
		*/
        controller_handle_namespace.assign("barrett/wam_dmp_ik_controller");
        return true;
    }    

    bool DMPIKController::readParameters()
    {
        std::string controller_handle_namespace;
        ROS_VERIFY(getArmRelatedVariables(node_handle_.getNamespace(), controller_handle_namespace));
        ros::NodeHandle controller_handle(controller_handle_namespace);
        ROS_VERIFY(usc_utilities::read(controller_handle, std::string("root_name"), root_name_));
        usc_utilities::appendLeadingSlash(root_name_);
        ROS_VERIFY(usc_utilities::read(controller_handle, std::string("keep_restposture_fixed_for_testing"), keep_restposture_fixed_for_testing_));
        ROS_VERIFY(usc_utilities::read(node_handle_, std::string("publisher_buffer_size"), publisher_buffer_size_));
        return true;
    }

    bool DMPIKController::initRTPublisher()
    {
        viz_marker_actual_arrow_publisher_.reset(new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(node_handle_, "dmp_ik_controller_marker_actual_arrow", publisher_buffer_size_));

        viz_marker_desired_arrow_publisher_.reset(new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(node_handle_, "dmp_ik_controller_marker_desired_arrow", publisher_buffer_size_));

        viz_marker_actual_line_publisher_.reset(new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(node_handle_, "dmp_ik_controller_marker_actual_line", publisher_buffer_size_));

        viz_marker_desired_line_publisher_.reset(new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(node_handle_, "dmp_ik_controller_marker_desired_line", publisher_buffer_size_));        

        geometry_msgs::Point actual_point;
        actual_line_points_.reset(new CircularMessageBuffer<geometry_msgs::Point>(visualization_line_max_points_, actual_point));

        geometry_msgs::Point desired_point;
        desired_line_points_.reset(new CircularMessageBuffer<geometry_msgs::Point>(visualization_line_max_points_, desired_point));

        pose_actual_publisher_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(node_handle_, "dmp_pose_actual", publisher_buffer_size_));

        pose_desired_publisher_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(node_handle_, "dmp_pose_desired", publisher_buffer_size_));
        
        
        return true;
    }    

    bool DMPIKController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &node_handle)
    {
        node_handle_ = node_handle;

        ROS_VERIFY(readParameters());
        ROS_VERIFY(initRTPublisher());

        std::vector<std::string> robot_parts;
        ROS_VERIFY(usc_utilities::read(node_handle_, "robot_parts", robot_parts));

        std::vector<std::string> joint_names;
        ROS_VERIFY(robot_info::RobotInfo::getArmJointNames(robot_parts, joint_names));

        num_joints_ = (int)joint_names.size();
        ROS_INFO("Initializing DMP IK Controller with >%i< joints.", num_joints_);
        int num_dof = usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT + num_joints_; // for the whole WAM arm, 14

        desired_positions_ = Eigen::VectorXd::Zero(num_dof);
        desired_velocities_ = Eigen::VectorXd::Zero(num_dof);
        desired_accelerations_ = Eigen::VectorXd::Zero(num_dof);

        goal_ = Eigen::VectorXd::Zero(num_dof);
        start_ = Eigen::VectorXd::Zero(num_dof);
        local_vector_ = Eigen::VectorXd::Zero(num_dof);

        // Get the dmp controller definition 
        std::string class_name;
        ROS_VERIFY(usc_utilities::read(node_handle_, "dmp_implementation", class_name));
        if (class_name == "NC2010DMPControllerImplementation")
        {
            dmp_controller_.reset(new DMPControllerImplementation<dmp::NC2010DMP>());
        }
        else 
        {
            ROS_ERROR("Could not find the implementation of DMPController %s.", class_name.c_str());
            return (initialized_  = false);
        }

        // initialize the dmp controller 
        std::vector<std::string> controller_variable_names;
        ROS_VERIFY(usc_utilities::read(node_handle_, "trajectory/variable_names", controller_variable_names));
        ROS_VERIFY(dmp_controller_->initialize(node_handle_.getNamespace(), controller_variable_names));

        // initiaze the cartesian controller 
        cart_controller_.reset(new CartController());
        
        if (!cart_controller_->init(hw, node_handle_))
        {
            ROS_ERROR("Could not initialize Cartesian Controller.");
            return (initialized_ = false);
        }
        
        
        ROS_INFO("Done initializing DMP IK controller.");
        return (initialized_ = true);
    }

    bool DMPIKController::initXml(hardware_interface::EffortJointInterface *hw,
                                  TiXmlElement* config)
    {
        ros::NodeHandle node_handle(config->Attribute("name"));
        return init(hw, node_handle);
    }

    // REAL-TIME REQUIREMENTS 
    void DMPIKController::starting(const ros::Time& time)
    {
        first_time_ = true;
        execution_error_ = false;

        dmp_controller_->stop(); // this resets the dmp controller.
        
        if (!holdPositions())
        {
            ROS_ERROR("Problem when holding position when starting DMP IK controller. (Real-time violation)");
            execution_error_ = true;
        }
        cart_controller_->starting(time);        
    }

    // REAL-TIME REQUIREMENTS
    void DMPIKController::stopping(const ros::Time& time)
    {
        cart_controller_->stopping(time);
    }

    // REAL-TIME REQUIREMENTS
    void DMPIKController::update(const ros::Time& time, const ros::Duration& period)
    {
        if (execution_error_)
        {
            ROS_ERROR("Execution Error!");
            return;
        }

        if (dmp_controller_->newDMPReady())
        {
            dmp_lib::DMPBasePtr dmp;
            if (!dmp_controller_->getDMP(dmp))
            {
                ROS_ERROR("Could not get the DMP. This should never happen (Real-time Violation).");
                execution_error_ = true;
                return;
            }

            if (!dmp->getGoal(goal_))
            {
                ROS_ERROR("Could not get the DMP goal point. This should never happen(Real-time Violation).");
                execution_error_ = true;
                return;
            }

            getDesiredPosition();
            if(!adjustVariables(desired_positions_, start_))
            {
                ROS_ERROR("Could not rearange DMP variables (Real-time Violatio)!");
                execution_error_ = true;
                return;
            }

            if (!dmp->changeStart(start_))
            {
                ROS_ERROR("Could not get start of the DMP (Real-time Violation)!");
                execution_error_ = true;
                return;
            }
        }

        // integrate DMP 
        if (dmp_controller_->isRunning(desired_positions_, desired_velocities_, desired_accelerations_))
        {
            if (!setDesiredState()) // Set the corresponding variables in cart_controller 
            {
                ROS_ERROR("Could not set desired states (Real-time Violation)!");
                execution_error_ = true;
                return;
            }
        }

        else 
        {
            if (!holdPositions())
            {
                ROS_ERROR("Failed to hold positions (Real-time Violation)!");
                execution_error_ = true;
                return;
            }
        }

        //visualize();
        cart_controller_->update(time, period);
    }        

    // REAL-TIME REQUIREMENTS
    bool DMPIKController::setDesiredState()
    {
        int num_used_variables;
        double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
        int num_quat_set = 0;

        if (dmp_controller_->getNumUsedVariables(num_used_variables))
        {
            // set only those variables that are used 
            for (int i = 0; i < num_used_variables; i++)
            {
                int index;
                if (!dmp_controller_->getVariableNameMap().getSupportedVariableIndex(i, index))
                {
                    ROS_ERROR("Could not get index >%i<.", i);
                    return false;
                }
                index++;
                int local_index = index - 1;

                //***************************************************
                // Set desired positions and linear velociteis 
                if ((local_index >= 0) && (local_index < usc_utilities::Constants::N_CART))
                {
                    cart_controller_->kdl_pose_desired_.p(local_index) = desired_positions_(local_index);
                    if (local_index == usc_utilities::Constants::X)
                    {
                        cart_controller_->kdl_twist_desired_.vel.x(desired_velocities_(local_index));
                    }
                    else if (local_index == usc_utilities::Constants::Y)
                    {
                        cart_controller_->kdl_twist_desired_.vel.y(desired_velocities_(local_index));
                    }
                    else if (local_index == usc_utilities::Constants::Z)
                    {
                        cart_controller_->kdl_twist_desired_.vel.z(desired_velocities_(local_index));
                    }
                }

                else if ((local_index >= usc_utilities::Constants::N_CART) && (local_index < usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT))
                {
                    // Set desired orientation and angular velocities 
                    if (local_index == usc_utilities::Constants::N_CART + usc_utilities::Constants::QW)
                    {
                        num_quat_set++;
                        qw = desired_positions_(local_index);
                    }

                    else if (local_index == usc_utilities::Constants::N_CART + usc_utilities::Constants::QX)
                    {
                        num_quat_set++;
                        qx = desired_positions_(local_index);
                        cart_controller_->kdl_twist_desired_.rot.x(desired_velocities_(local_index));
                    }

                    else if (local_index == usc_utilities::Constants::N_CART + usc_utilities::Constants::QY)
                    {
                        num_quat_set++;
                        qy = desired_positions_(local_index);
                        cart_controller_->kdl_twist_desired_.rot.y(desired_velocities_(local_index));
                    }

                    else if (local_index == usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ)
                    {
                        num_quat_set++;
                        qz = desired_positions_(local_index);
                        cart_controller_->kdl_twist_desired_.rot.z(desired_velocities_(local_index));
                    }
                }

                // set desired nullspace posture 
                else if ((local_index >= usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT) && (local_index < usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT + num_joints_))
                {
                    int joint_index = local_index - (usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT);
                    cart_controller_->rest_posture_joint_configuration_(joint_index) = desired_positions_(local_index);
                }

                else 
                {
                    ROS_ERROR("Unknown index >%i< (Real-time violation)!", index);
                    return false;
                }
            }
        }

        else 
        {
            // Set endeffector positions, linear valocities, and linear accelerations 
            for (int i = usc_utilities::Constants::X; i <= usc_utilities::Constants::Z; i++)
            {
                cart_controller_->kdl_pose_desired_.p(i) = desired_positions_(i);
            }

            cart_controller_->kdl_twist_desired_.vel.x(desired_velocities_(usc_utilities::Constants::X));
            cart_controller_->kdl_twist_desired_.vel.y(desired_velocities_(usc_utilities::Constants::Y));
            cart_controller_->kdl_twist_desired_.vel.z(desired_velocities_(usc_utilities::Constants::Z));
            
            // Set endeffector orientation, angular velociteis, and angular accelerations 
            qw = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QW);
            qx = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QX);
            qy = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QY);
            qz = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ);
            num_quat_set = usc_utilities::Constants::N_QUAT;

            // Set angular velocities 
            cart_controller_->kdl_twist_desired_.rot.x(desired_velocities_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QX));
            cart_controller_->kdl_twist_desired_.rot.y(desired_velocities_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QY));
            cart_controller_->kdl_twist_desired_.rot.z(desired_velocities_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ));

            // Set desired nullspace posture 
            for (int i = 0; i < num_joints_; i++)
            {
                cart_controller_->rest_posture_joint_configuration_(i) = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT + i);
            }
        }

        if ((num_quat_set != 0) && (num_quat_set != usc_utilities::Constants::N_QUAT))
        {
            ROS_ERROR("Missing quaternion number. There are only >%i<. Cannot set orientation!", num_quat_set);
            return false;
        }
        cart_controller_->kdl_pose_desired_.M = KDL::Rotation::Quaternion(qx, qy, qz, qw);

        return true;
    }

    // REAL-TIME REQUIREMENTS
    bool DMPIKController::adjustVariables(const Eigen::VectorXd& input_vector,
                                          Eigen::VectorXd& output_vector)
    {
        if (input_vector.size() != output_vector.size())
        {
            ROS_ERROR("Size of the input vector >%i< must equal the size of the output vector >%i< (Real-time violation)!", (int)(input_vector.size()), (int)(output_vector.size()));
            return false;
        }
        local_vector_ = input_vector;

        int num_used_variables;
        if (dmp_controller_->getNumUsedVariables(num_used_variables))
        {
            // Set only those variables that are used 
            for (int i = 0; i < num_used_variables; i++)
            {
                int index;
                if (!dmp_controller_->getVariableNameMap().getSupportedVariableIndex(i, index))
                {
                    ROS_ERROR("Could not get index >%i< to remap variables (Real-time violation)!", i);
                    return false;
                }
                output_vector(i) = local_vector_(index);
            }
        }
        return true;
    }

    // REAL-TIME REQUIREMENTS
    bool DMPIKController::holdPositions()
    {
        if (!getDesiredPosition())
        {
            return false;
        }

        for (int i = 0; i < desired_velocities_.size(); i++)
        {
            desired_velocities_(i) = 0.0;
            desired_accelerations_(i) = 0.0;
        }

        if (!setDesiredState())
        {
            return false;
        }
        return true;
    }

    // REAL-TIME REQUIREMENTS
    bool DMPIKController::getDesiredPosition()
    {
        for (int i = usc_utilities::Constants::X; i <= usc_utilities::Constants::Z; i++)
        {
            desired_positions_(i) = cart_controller_->kdl_pose_desired_.p(i);
        }
        desired_velocities_(usc_utilities::Constants::X) = cart_controller_->kdl_twist_desired_.vel.x();
        desired_velocities_(usc_utilities::Constants::Y) = cart_controller_->kdl_twist_desired_.vel.y();
        desired_velocities_(usc_utilities::Constants::Z) = cart_controller_->kdl_twist_desired_.vel.z();

        double qw, qx, qy, qz;
        cart_controller_->kdl_pose_desired_.M.GetQuaternion(qx, qy, qz, qw);
        desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QX) = qx;
        desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QY) = qy;
        desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ) = qz;
        desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QW) = qw;

        // Get angular velocities 
        desired_velocities_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QX) = cart_controller_->kdl_twist_desired_.rot.x();
        desired_velocities_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QY) = cart_controller_->kdl_twist_desired_.rot.y();
        desired_velocities_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ) = cart_controller_->kdl_twist_desired_.rot.z();

        // Get desired nullspace posture 
        for (int i = 0; i < num_joints_; i++)
        {
            desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT + i) = cart_controller_->rest_posture_joint_configuration_(i);
        }

        return true;
    }

    // REAL-TIME REQUIREMENTS
    void DMPIKController::visualize()
    {
        publishing_counter_++;
        if (publishing_counter_ % publishing_rate_ == 0)
        {
            publishing_counter_ = 0;

            if (pose_actual_publisher_ && pose_actual_publisher_->trylock())
            {
                pose_actual_publisher_->msg_.header.frame_id = root_name_;
                pose_actual_publisher_->msg_.header.stamp = ros::Time::now();
                pose_actual_publisher_->msg_.header.seq = 0;
                pose_actual_publisher_->msg_.pose.position.x = cart_controller_->kdl_real_pose_measured_.p.x();
                pose_actual_publisher_->msg_.pose.position.y = cart_controller_->kdl_real_pose_measured_.p.y();
                pose_actual_publisher_->msg_.pose.position.z = cart_controller_->kdl_real_pose_measured_.p.z();
                double qx, qy, qz, qw;
                cart_controller_->kdl_real_pose_measured_.M.GetQuaternion(qx, qy, qz, qw);
                pose_actual_publisher_->msg_.pose.orientation.x = qx;
                pose_actual_publisher_->msg_.pose.orientation.y = qy;
                pose_actual_publisher_->msg_.pose.orientation.z = qz;
                pose_actual_publisher_->msg_.pose.orientation.w = qw;
                pose_actual_publisher_->unlockAndPublish();
            }
            else 
            {
                ROS_ERROR("Skipping actual pose visualization (Real-time violation)!");
            }

            if (pose_desired_publisher_ && pose_desired_publisher_->trylock())
            {
                pose_desired_publisher_->msg_.header.frame_id = root_name_;
                pose_desired_publisher_->msg_.header.stamp = ros::Time::now();
                pose_desired_publisher_->msg_.header.seq = 0;
                pose_desired_publisher_->msg_.pose.position.x = desired_positions_(usc_utilities::Constants::X);
                pose_desired_publisher_->msg_.pose.position.y = desired_positions_(usc_utilities::Constants::Y);
                pose_desired_publisher_->msg_.pose.position.z = desired_positions_(usc_utilities::Constants::Z);
                pose_desired_publisher_->msg_.pose.orientation.w = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QW);
                pose_desired_publisher_->msg_.pose.orientation.x = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QX);
                pose_desired_publisher_->msg_.pose.orientation.y = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QY);
                pose_desired_publisher_->msg_.pose.orientation.z = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ);

                pose_desired_publisher_->unlockAndPublish();
            }
            else 
            {
                ROS_ERROR("Skipping the desired pose visualization (Realtime-violation)!");
            }
            
            

            
            Eigen::Matrix<double, 3, 1> velocity_vec;
            Eigen::Matrix<double, 3, 1> world_vec;
            Eigen::Quaternion<double> eigen_quat;
            
            if (viz_marker_actual_arrow_publisher_ && viz_marker_actual_arrow_publisher_->trylock())
            {
                viz_marker_actual_arrow_publisher_->msg_.header.frame_id = root_name_;
                viz_marker_actual_arrow_publisher_->msg_.header.stamp = ros::Time::now();
                viz_marker_actual_arrow_publisher_->msg_.ns = "DMPActialArrow";
                viz_marker_actual_arrow_publisher_->msg_.type = visualization_msgs::Marker::ARROW;
                viz_marker_actual_arrow_publisher_->msg_.action = visualization_msgs::Marker::ADD;

                viz_marker_actual_arrow_publisher_->msg_.id = 1;
                viz_marker_actual_arrow_publisher_->msg_.scale.x = 0.8 * cart_controller_->kdl_twist_measured_(0);
                viz_marker_actual_arrow_publisher_->msg_.scale.y = 0.8 * cart_controller_->kdl_twist_measured_(1);
                viz_marker_actual_arrow_publisher_->msg_.scale.z = 0.8 * cart_controller_->kdl_twist_measured_(2);

                viz_marker_actual_arrow_publisher_->msg_.color.r = 0.0f;
                viz_marker_actual_arrow_publisher_->msg_.color.g = 0.0f;
                viz_marker_actual_arrow_publisher_->msg_.color.b = 1.0f;
                viz_marker_actual_arrow_publisher_->msg_.color.a = 0.5;

                viz_marker_actual_arrow_publisher_->msg_.lifetime = ros::Duration(); // Will never be deleted ?
                viz_marker_actual_arrow_publisher_->msg_.pose.position.x = cart_controller_->kdl_real_pose_measured_.p.x();
                viz_marker_actual_arrow_publisher_->msg_.pose.position.y = cart_controller_->kdl_real_pose_measured_.p.y();
                viz_marker_actual_arrow_publisher_->msg_.pose.position.z = cart_controller_->kdl_real_pose_measured_.p.z();

                double qx, qy, qz, qw;
                cart_controller_->kdl_real_pose_measured_.M.GetQuaternion(qx, qy, qz, qw);
                viz_marker_actual_arrow_publisher_->msg_.pose.orientation.x = qx;
                viz_marker_actual_arrow_publisher_->msg_.pose.orientation.y = qy;
                viz_marker_actual_arrow_publisher_->msg_.pose.orientation.z = qz;
                viz_marker_actual_arrow_publisher_->msg_.pose.orientation.w = qw;
                //viz_marker_actual_arrow_publisher_->unlockAndPublish(); 
                
                
                for (int i = 0; i < usc_utilities::Constants::N_CART; i++)
                {
                    velocity_vec(i) = cart_controller_->kdl_twist_measured_(i);
                    world_vec(i) = 0.0;
                }
                world_vec(usc_utilities::Constants::Z) = 1.0;
                eigen_quat.setFromTwoVectors(world_vec, velocity_vec);

                double length = velocity_vec.norm();
                if (length > 0.01)
                {
                    viz_marker_actual_arrow_publisher_->msg_.scale.x = length;
                }
                else 
                {
                    viz_marker_actual_line_publisher_->msg_.scale.x = 0.01;
                }
                viz_marker_actual_arrow_publisher_->msg_.scale.y = 0.25;
                viz_marker_actual_arrow_publisher_->msg_.scale.z = 0.25;

                if ((isinf(eigen_quat.x()) == 0) && (isnan(eigen_quat.x()) == 0) && (isinf(eigen_quat.y()) == 0) && (isnan(eigen_quat.y()) == 0) && (isinf(eigen_quat.z()) == 0) && (isnan(eigen_quat.z()) == 0) && (isinf(eigen_quat.w()) == 0) && (isnan(eigen_quat.w()) == 0))
                {
                    viz_marker_actual_arrow_publisher_->msg_.pose.orientation.x = eigen_quat.x();
                    viz_marker_actual_arrow_publisher_->msg_.pose.orientation.y = eigen_quat.y();
                    viz_marker_actual_arrow_publisher_->msg_.pose.orientation.z = eigen_quat.z();
                    viz_marker_actual_arrow_publisher_->msg_.pose.orientation.w = eigen_quat.w();
                    viz_marker_actual_arrow_publisher_->unlockAndPublish();                    
                }
                else 
                {
                    ROS_ERROR("Invalid quaternion coordinate! Cannot set orientation for the marker message (Real-time violation)!");
                    return;
                }
                
            }

            else 
            {
                ROS_ERROR("Skipping actual arrow visualization. (Real-time violation)!");
            }
            
            
            
            /*
            if (viz_marker_desired_arrow_publisher_ && viz_marker_desired_arrow_publisher_->trylock())
            {
                viz_marker_desired_arrow_publisher_->msg_.header.frame_id = root_name_;
                viz_marker_desired_arrow_publisher_->msg_.header.stamp = ros::Time::now();
                viz_marker_desired_arrow_publisher_->msg_.ns = "DMPDesiredArrow";
                viz_marker_desired_arrow_publisher_->msg_.type = visualization_msgs::Marker::ARROW;
                viz_marker_desired_arrow_publisher_->msg_.action = visualization_msgs::Marker::ADD;
                viz_marker_desired_arrow_publisher_->msg_.id = 2;
                viz_marker_desired_arrow_publisher_->msg_.scale.x = 0.8 * desired_velocities_(0);
                viz_marker_desired_arrow_publisher_->msg_.scale.y = 0.8 * desired_velocities_(1);
                viz_marker_desired_arrow_publisher_->msg_.scale.z = 0.8 * desired_velocities_(2);

                viz_marker_desired_arrow_publisher_->msg_.color.r = 0.0f;
                viz_marker_desired_arrow_publisher_->msg_.color.g = 1.0f;
                viz_marker_desired_arrow_publisher_->msg_.color.b = 0.0f;
                viz_marker_desired_arrow_publisher_->msg_.color.a = 0.5;

                viz_marker_desired_arrow_publisher_->msg_.lifetime = ros::Duration();

                viz_marker_desired_arrow_publisher_->msg_.pose.position.x = desired_positions_(usc_utilities::Constants::X);
                viz_marker_desired_arrow_publisher_->msg_.pose.position.y = desired_positions_(usc_utilities::Constants::Y);
                viz_marker_desired_arrow_publisher_->msg_.pose.position.z = desired_positions_(usc_utilities::Constants::Z);

                viz_marker_desired_arrow_publisher_->msg_.pose.orientation.x = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QX);
                viz_marker_desired_arrow_publisher_->msg_.pose.orientation.y = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QY);
                viz_marker_desired_arrow_publisher_->msg_.pose.orientation.z = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ);
                viz_marker_desired_arrow_publisher_->msg_.pose.orientation.w = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QW);
                for (int i = 0; i < usc_utilities::Constants::N_CART; i++)
                {
                    velocity_vec(i) = desired_velocities_(i);
                    world_vec(i) = 0.0;
                }
                world_vec(0) = 1.0;
                eigen_quat.setFromTwoVectors(world_vec, velocity_vec);

                double length = velocity_vec.norm();
                if (length > 0.01)
                {
                    viz_marker_desired_arrow_publisher_->msg_.scale.x = length;
                }
                else 
                {
                    viz_marker_desired_arrow_publisher_->msg_.scale.x = 0.01;
                }
                viz_marker_desired_arrow_publisher_->msg_.scale.y = 0.25;
                viz_marker_desired_arrow_publisher_->msg_.scale.z = 0.25;

                if ((isinf(eigen_quat.x()) == 0) && (isnan(eigen_quat.x()) == 0) && (isinf(eigen_quat.y()) == 0) && (isnan(eigen_quat.y()) == 0) && (isinf(eigen_quat.z()) == 0) && (isnan(eigen_quat.z()) == 0) && (isinf(eigen_quat.w()) == 0) && (isnan(eigen_quat.w()) == 0))
                {
                    viz_marker_desired_arrow_publisher_->msg_.pose.orientation.x = eigen_quat.x();
                    viz_marker_desired_arrow_publisher_->msg_.pose.orientation.y = eigen_quat.y();
                    viz_marker_desired_arrow_publisher_->msg_.pose.orientation.z = eigen_quat.z();
                    viz_marker_desired_arrow_publisher_->msg_.pose.orientation.w = eigen_quat.w();
                    viz_marker_desired_arrow_publisher_->unlockAndPublish();                
                }
                else 
                {
                    ROS_ERROR("Invalid quaternion coordinate! Cannot set orientation for the marker message (Real-time violation)!");
                    return;
                }
            }
            else 
            {
                ROS_ERROR("Skipping desired arrow visualization (Real-time violation)!");
            }
            */ 
            /* 
            visualization_line_counter_++;
            if (visualization_line_counter_ % visualization_line_rate_ == 0)
            {
                visualization_line_counter_ = 0;
                //publishing_seq_counter_++;

                if (viz_marker_actual_line_publisher_ && viz_marker_actual_line_publisher_->trylock())
                {
                    viz_marker_actual_line_publisher_->msg_.header.frame_id = root_name_;
                    viz_marker_actual_line_publisher_->msg_.header.stamp = ros::Time::now();
                    viz_marker_actual_line_publisher_->msg_.header.seq = publishing_seq_counter_;
                    viz_marker_actual_line_publisher_->msg_.ns = "DMPActualLine";
                    viz_marker_actual_line_publisher_->msg_.type = visualization_msgs::Marker::LINE_STRIP;
                    viz_marker_actual_line_publisher_->msg_.id = 3;
                    viz_marker_actual_line_publisher_->msg_.scale.x = 0.05;
                    viz_marker_actual_line_publisher_->msg_.scale.y = 0.05;
                    viz_marker_actual_line_publisher_->msg_.scale.z = 0.05;
                    viz_marker_actual_line_publisher_->msg_.lifetime = ros::Duration();
                    viz_marker_actual_line_publisher_->msg_.color.r = 0.0f;
                    viz_marker_actual_line_publisher_->msg_.color.g = 0.0f;
                    viz_marker_actual_line_publisher_->msg_.color.b = 1.0f;
                    viz_marker_actual_line_publisher_->msg_.color.a = 0.4;

                    geometry_msgs::Point point;
                    point.x = cart_controller_->kdl_real_pose_measured_.p.x();
                    point.y = cart_controller_->kdl_real_pose_measured_.p.y();
                    point.z = cart_controller_->kdl_real_pose_measured_.p.z();

                    viz_marker_actual_line_publisher_->msg_.points.resize(visualization_line_max_points_);

                    if (first_time_)
                    {
                        for (int i = 0; i < visualization_line_max_points_; i++)
                        {
                            actual_line_points_->push_front(point);
                        }
                    }
                    actual_line_points_->push_front(point);
                    if (!actual_line_points_->get(viz_marker_actual_line_publisher_->msg_.points))
                    {
                        ROS_ERROR("Cannot get actual line points from the circular message buffer. (Real-time violation)!");
                    } 
                    
                    viz_marker_actual_line_publisher_->unlockAndPublish();
                }       
                else 
                {
                    ROS_ERROR("Skipping actual line visualization (Real-time violation)!");
                }

                //publishing_seq_counter_++; 

                if (viz_marker_desired_line_publisher_ && viz_marker_desired_line_publisher_->trylock())
                {
                    viz_marker_desired_line_publisher_->msg_.header.frame_id = root_name_;
                    viz_marker_desired_line_publisher_->msg_.header.stamp = ros::Time::now();
                    viz_marker_desired_line_publisher_->msg_.header.seq = publishing_seq_counter_;
                    viz_marker_desired_line_publisher_->msg_.ns = "DMPDesiredLine";
                    viz_marker_desired_line_publisher_->msg_.type = visualization_msgs::Marker::LINE_STRIP;
                    viz_marker_desired_line_publisher_->msg_.id = 4;
                    viz_marker_desired_line_publisher_->msg_.scale.x = 0.05;
                    viz_marker_desired_line_publisher_->msg_.scale.y = 0.05;
                    viz_marker_desired_line_publisher_->msg_.scale.z = 0.05;
                    viz_marker_desired_line_publisher_->msg_.lifetime = ros::Duration();
                    viz_marker_desired_line_publisher_->msg_.color.r = 0.0f;
                    viz_marker_desired_line_publisher_->msg_.color.g = 1.0f;
                    viz_marker_desired_line_publisher_->msg_.color.b = 0.0f;
                    viz_marker_desired_line_publisher_->msg_.color.a = 0.4;

                    geometry_msgs::Point point;
                    point.x = desired_positions_(usc_utilities::Constants::X);
                    point.y = desired_positions_(usc_utilities::Constants::Y);
                    point.z = desired_positions_(usc_utilities::Constants::Z);

                    viz_marker_desired_line_publisher_->msg_.points.resize(visualization_line_max_points_);
                    
                    if (first_time_)
                    {
                        for (int i = 0; i < visualization_line_max_points_; i++)
                        {
                            desired_line_points_->push_front(point);
                        }
                    }
                    desired_line_points_->push_front(point);
                    if (!desired_line_points_->get(viz_marker_desired_line_publisher_->msg_.points))
                    {
                        ROS_ERROR("Cannot get actual line points from the circular message buffer. (Real-time violation)!");
                    }
                    
                    viz_marker_desired_line_publisher_->unlockAndPublish();
                }
                else 
                {
                    ROS_ERROR("Skipping desired line visualization (Real-time violation)!");
                }
                first_time_ = false;
            }
            */
            /* viz_marker_desired_line_publisher_.msg_.points.resize(visualization_line_max_points_)
             */
        }
    }
}
PLUGINLIB_EXPORT_CLASS(wam_dmp_controller::DMPIKController, controller_interface::ControllerBase)

