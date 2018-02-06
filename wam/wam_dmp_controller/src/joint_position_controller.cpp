/*************************************************************************
	> File Name: joint_position_controller.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 05 Jun 2017 03:51:57 PM PDT
 ************************************************************************/

#include <wam_dmp_controller/joint_position_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace wam_dmp_controller
{
    JointPositionController::JointPositionController() : initialized_(false), error_(0.0), error_dot_(0.0), last_error_(0), commanded_effort_(0.0), publisher_rate_(10), publisher_counter_(0), publisher_buffer_size_(1000), last_time_(0)
    {
    }

    JointPositionController::~JointPositionController()
    {
        sub_command_.shutdown();
    }

    bool JointPositionController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
        assert(hw);
        // Get joint name from a prameter server 
        std::string joint_name;
        if (!n.getParam("joint", joint_name))
        {
            ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
            initialized_ = false;
            return initialized_;
        }

        // Load PID Controller using gains set on parameter server 
        if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
        {
            ROS_ERROR("Could not initialize a pid controller for joint %s.", n.getNamespace().c_str());
            initialized_ = false;
            return initialized_;
        }

        // Start the realtime publisher 
        controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<wam_dmp_controller::JointControllerState>(n, "controller_state", 1));

        // Start command subscriber 
        sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointPositionController::setCommandCB, this);

        // Get joint handle from the hardware interface 
        joint_ = hw->getHandle(joint_name);

        // Get URDF info about the joint 
        std::string urdf_str;
		ros::NodeHandle temp_nh("barrett"); //TODO be careful of the namespace here
        ROS_VERIFY(usc_utilities::read(temp_nh, "robot_description_yi", urdf_str));
        urdf::Model urdf;
        //if (!urdf.initParamWithinNodeHandle("robot_description", n))
        if (!urdf.initString(urdf_str))
        {
            ROS_ERROR("Failed to parse the urdf file");
            initialized_ = false;
            return initialized_;
        }
        joint_urdf_ = urdf.getJoint(joint_name);
        if (!joint_urdf_)
        {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
            initialized_ = false;
            return initialized_;
        }

        initialized_ = true;
        return initialized_;
    }

    void JointPositionController::setGains(const double &p,
                                      const double &i,
                                      const double &d,
                                      const double &i_max,
                                      const double &i_min,
                                      const bool &antiwindup)
    {
        pid_controller_.setGains(p, i, d, i_max, i_min);//, antiwindup);
    }

    void JointPositionController::getGains(double &p,
                                           double &i,
                                           double &d,
                                           double &i_max,
                                           double &i_min,
                                           bool &antiwindup)
    {
        pid_controller_.getGains(p, i, d, i_max, i_min);
    }

    void JointPositionController::getGains(double &p,
                                           double &i, 
                                           double &d,
                                           double &i_max,
                                           double &i_min)
    {
        bool dummy;
        pid_controller_.getGains(p, i, d, i_max, i_min);
    }

    void JointPositionController::printDebug()
    {
        pid_controller_.printValues();
    }

    std::string JointPositionController::getJointName()
    {
        return joint_.getName();
    }

    void JointPositionController::setCommand(double pos_target)
    {
        command_struct_.position_ = pos_target;
        command_struct_.has_velocity_ = false; // Flag to ignore the velocity command since out setCommand method did not include it 

        // the writeFromNonRT can be used in RT, if you have the gurantee that
        // no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
        // there is only one single rt thread 
        command_.writeFromNonRT(command_struct_);
    }

    void JointPositionController::setCommand(double pos_target, double vel_target)
    {
        command_struct_.position_ = pos_target;
        command_struct_.velocity_ = vel_target;
        command_struct_.has_velocity_ = true;

        command_.writeFromNonRT(command_struct_);
    }

    void JointPositionController::starting(const ros::Time& time)
    {
        double pos_command = joint_.getPosition();

        // Make sure the joint is within limits if applicable 
        enforceJointLimits(pos_command);

        command_struct_.position_ = pos_command;
        command_struct_.has_velocity_ = false;
        command_.initRT(command_struct_);

        pid_controller_.reset();
    }

    void JointPositionController::update(const ros::Time& time, const ros::Duration& period)
    {
        command_struct_ = *(command_.readFromRT());
        double command_position = command_struct_.position_;
        double command_velocity = command_struct_.velocity_;
        bool has_velocity_ = command_struct_.has_velocity_;

        double current_position = joint_.getPosition();

        // Make sure the joint is within limits if applicable
        enforceJointLimits(command_position);

        // Compute the position error 
        if (joint_urdf_->type == urdf::Joint::REVOLUTE)
        {
            angles::shortest_angular_distance_with_limits(current_position, command_position, joint_urdf_->limits->lower, joint_urdf_->limits->upper, error_);
        }
        else if (joint_urdf_->type == urdf::Joint::CONTINUOUS)
        {
            error_ = angles::shortest_angular_distance(current_position, command_position);
        }
        else // Prismatic joint 
        {
            error_ = command_position - current_position;
        }

        // Decide which of the two PID computeCommand() methods to call 
        if (has_velocity_)
        {
            // Compute velocity error if a non-zero velocity command was given 
            error_dot_ = command_velocity - joint_.getVelocity();

            // Set the PID error and compute the PID command with non-uniform time step size.
            // This also allows the user to pass in a precomputed derivative error.
            commanded_effort_ = pid_controller_.computeCommand(error_, error_dot_, period);
        }
        else
        {
            // Set the PID error and compute the PID command with non-uniform time step size.
            commanded_effort_ = pid_controller_.computeCommand(error_, period);
        }

        joint_.setCommand(commanded_effort_);
        last_time_ = time;

        // publish rate 
        if (publisher_counter_ % publisher_rate_ == 0)
        {
            if (controller_state_publisher_ && controller_state_publisher_->trylock())
            {
                controller_state_publisher_->msg_.header.stamp = time;
                controller_state_publisher_->msg_.set_point = command_position;
                controller_state_publisher_->msg_.process_value = current_position;
                controller_state_publisher_->msg_.process_value_dot - joint_.getVelocity();
                controller_state_publisher_->msg_.error = error_;
                controller_state_publisher_->msg_.time_step = period.toSec();
                controller_state_publisher_->msg_.command = commanded_effort_;

                double dummy;
                bool antiwindup;
                getGains(controller_state_publisher_->msg_.p,
                         controller_state_publisher_->msg_.i,
                         controller_state_publisher_->msg_.d,
                         controller_state_publisher_->msg_.i_clamp,
                         dummy,
                         antiwindup);
                controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
                controller_state_publisher_->unlockAndPublish();
            }
        }
        publisher_counter_++;
    }

	void JointPositionController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
	{
  		setCommand(msg->data);
	}


    void JointPositionController::enforceJointLimits(double &command)
    {
        if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
        {
            if (command > joint_urdf_->limits->upper)
            {
                command = joint_urdf_->limits->upper;
            }
            else if (command < joint_urdf_->limits->lower)
            {
                command = joint_urdf_->limits->lower;
            }
        }
    }
}

PLUGINLIB_EXPORT_CLASS(wam_dmp_controller::JointPositionController, controller_interface::ControllerBase)

