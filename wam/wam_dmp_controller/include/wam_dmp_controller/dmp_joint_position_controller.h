/*************************************************************************
	> File Name: dmp_joint_position_controller.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 06 Jun 2017 01:21:41 PM PDT
 ************************************************************************/

#ifndef _DMP_JOINT_POSITION_CONTROLLER_H
#define _DMP_JOINT_POSITION_CONTROLLER_H
// system include 
#include <boost/shared_ptr.hpp>
#include <dynamic_movement_primitive/dynamic_movement_primitive.h>

// ros include 
#include <ros/ros.h>
#include <control_toolbox/pid.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

// local include 
#include <wam_dmp_controller/joint_position_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <wam_dmp_controller/dmp_controller.h>

#include <Eigen/Eigen>
#include <vector>

namespace wam_dmp_controller
{
    class DMPJointPositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            DMPJointPositionController() {};
            virtual ~DMPJointPositionController() {};

            /*!
             * @param hw the specific hardware interface used by the controller 
             * @param node A NodeHandle where the controller should read its configuration 
             */
            bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &node_handle);

            void starting(const ros::Time &time);
            void update(const ros::Time &time, const ros::Duration &period);

            /*!
             * REAL-TIME REQUIREMENTS
             */
            void setDesiredState();

            /*!
             * REAL-TIME REQUIREMENTS
             */
            void holdPositions();

            void getDesiredPosition();

        private:
            
            hardware_interface::JointHandle joint_;
            int num_joints_;

            Eigen::VectorXd desired_positions_;
            Eigen::VectorXd desired_velocities_;
            Eigen::VectorXd desired_accelerations_;

            std::vector<boost::shared_ptr<JointPositionController> > joint_position_controllers_; // use boost::shared_ptr to get around copy constructor weirdness
            //std::vector<JointPositionController> joint_position_controllers_;
            
            boost::shared_ptr<DMPController> dmp_controller_;
    };
}
#endif
