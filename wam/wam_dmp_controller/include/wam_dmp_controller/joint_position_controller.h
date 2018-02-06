/*************************************************************************
	> File Name: joint_position_controller.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 05 Jun 2017 12:22:05 PM PDT
 ************************************************************************/

#ifndef _JOINT_POSITION_CONTROLLER_H
#define _JOINT_POSITION_CONTROLLER_H

/**
 * @class effort_controllers::JointPositionCOntroller
 * @brief Joint Position Controller 
 *
 * This class controls posiiotn using a pid loop 
 * section ROS ROS interface 
 *
 * @oaram type Must be "effort_controllers::JointPOsitionController"
 * @param joint Name of the joint to control.
 * @param pid Contains the gains for the PID loop around position, See: control_toolbox::Pid 
 *
 * Subscribe to:
 *  @b command (std_msgs::Float64) : The joint position to achieve 
 *
 * Publishes:
 *  @b state (control_msgs::JointControllerState) : Current state of the controller, including pid errors and gains.
 *
 */

#include <ros/node_handle.h>
 
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <control_toolbox/pid.h>
#include <urdf/model.h>
#include <angles/angles.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <std_msgs/Float64.h>
#include <wam_dmp_controller/JointControllerState.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

namespace wam_dmp_controller
{
    class JointPositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:
            
            /**
             * \brief Store position and velocity command in struct to allow easier realtime buffer usage 
             */
            struct Commands
            {
                double position_; // Last commanded position 
                double velocity_; // Last commanded velocity 
                bool has_velocity_; // false if no velocity command has been specified 
            };

            JointPositionController();
            ~JointPositionController();

            /** \brief the init function is called to initialize the controller from a 
             * non-realtime thread with a pointer to the hardware interface itself, 
             * instead of a pointer to a RobotHW
             *
             * \param robot The specific hardware interface used by the controller 
             * 
             * \param n A NodeHandle in the namespace from which the controller should
             * read its configuration, and where it should set up its ROS interface.
             *
             * \returns True if initialization was successful and the controller is ready to be started 
             */
            bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n);

            /*!
             * \brief Gve set position of the joint for next update: revolute (angle) and prismatic (position)
             *
             * \param command 
             */
            void setCommand(double pos_target);

            /*!
             * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
             * Also supports a target velocity
             *
             * \param pos_target - position setpoint
             * \param vel_target - velocity setpoint
             */
            void setCommand(double pos_target, double vel_target);

            /*!
             * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
             */
            void getCommand(double &pos_target);

            /*!
             * \brief Get set position of the joint for next update: revolute (angle) and prismatic (position)
             * Also supports a target velocity
             *
             * \param pos_target - position setpoint
             * \param vel_target - velocity setpoint
             */
            void getCommand(double &pos_target, double &vel_target);

            /** \brief This is called from within the realtime thread just before the 
             * first call to \ref update 
             *
             * \param time The current time 
             */
            void starting(const ros::Time& time);

            /*!
             * \brief Issues commands to the joint. Should be called ar regular intervals 
             */
            void update(const ros::Time& time, const ros::Duration& period);

            /**
             * \brief Get the PID parameters 
             */
            void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

            /**
             * \brief Get the PID parameters 
             */
            void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);

            /**
             * \brief Set the PID parameters 
             */
            void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);

            /**
             * \brief Print debug info to the console 
             */
            void printDebug();

            /**
             * \brief Get the name of the joint its controller uses 
             */
            std::string getJointName();

            /**
             * \brief Get the current position of the joint 
             * \return current positoin 
             */
            double getJointPosition() const; 

            /**
             * \brief Get the current velocity of the joint 
             * \return current velocity 
             */
            double getJointVelocity() const; 

            double getJointPositionError() const;
            double getJointVelocityError() const;

            double getCommandedEffort() const;

            int getJointType() const;

            hardware_interface::JointHandle joint_;
            boost::shared_ptr<const urdf::Joint> joint_urdf_;
            realtime_tools::RealtimeBuffer<Commands> command_;
            Commands command_struct_; // pre-allocated member that is re-used to set the realtime buffer

        private:
            bool initialized_;

            control_toolbox::Pid pid_controller_; /**<Internal PID controller. */

            boost::scoped_ptr<realtime_tools::RealtimePublisher<wam_dmp_controller::JointControllerState> > controller_state_publisher_;

            ros::Subscriber sub_command_;

            double error_, error_dot_, last_error_, commanded_effort_;
            int publisher_rate_, publisher_counter_, publisher_buffer_size_;

            ros::NodeHandle node_handle_;
            ros::Time last_time_;

            void publish();

            /**
             * \brief Callback from /command subscriber for setpoint 
             */
            void setCommandCB(const std_msgs::Float64ConstPtr& msg);

            /**
             * \brief Check that the command is within the hard limits of the joint. Checks for joint 
             * type first. Sets command to limit if out of bounds.
             * \param command - the input to test 
             */
            void enforceJointLimits(double &command);
    };
    
    inline double JointPositionController::getJointPosition() const
    {
        if (joint_urdf_->type == urdf::Joint::CONTINUOUS)
        {
            return angles::normalize_angle(joint_.getPosition());
        }
        return joint_.getPosition();
    }

    inline double JointPositionController::getJointVelocity() const 
    {
        return joint_.getVelocity();
    }

    inline double JointPositionController::getJointPositionError() const 
    {
        return error_;
    }

    inline double JointPositionController::getJointVelocityError() const 
    {
        return error_dot_;
    }

    inline double JointPositionController::getCommandedEffort() const 
    {
        return commanded_effort_;
    }

    inline int JointPositionController::getJointType() const 
    {
        return joint_urdf_->type;
    }
}

#endif
