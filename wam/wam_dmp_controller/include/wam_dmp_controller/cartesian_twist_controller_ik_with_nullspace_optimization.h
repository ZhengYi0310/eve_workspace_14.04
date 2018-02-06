/*************************************************************************
	> File Name: cartesian_twist_controller_ik_with_nullspace_optimization.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 09 Jun 2017 03:08:24 PM PDT
 ************************************************************************/

#ifndef _CARTESIAN_TWIST_CONTROLLER_IK_WITH_NULLSPACE_OPTIMIZATION_H
#define _CARTESIAN_TWIST_CONTROLLER_IK_WITH_NULLSPACE_OPTIMIZATION_H

// system includes 
#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

// ros includes 
#include <ros/ros.h>
#include <rosrt/rosrt.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

#include <filters/transfer_function.h>

// Eigen includes 
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/LU>

// local includes 
#include <wam_dmp_controller/joint_position_controller.h>
#include <wam_dmp_controller/JointPositionVelocityStamped.h>
#include <wam_dmp_controller/PoseTwistStamped.h>
#include <wam_dmp_controller/NullspaceTermStamped.h>

namespace wam_dmp_controller
{
    class CartesianTwistControllerWithNullspaceOptimization : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            CartesianTwistControllerWithNullspaceOptimization();
            virtual ~CartesianTwistControllerWithNullspaceOptimization() {};

            /**
             * @param hw the specific hardware interface 
             * @node_handle a node_handle in the namespace from which the controller should read its configuration
             */
            bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &node_handle);

            void starting(const ros::Time& time);

            void update(const ros::Time& time, const ros::Duration& period);
    
            /*! input to the cartesian twist controller 
             */
            KDL::Frame kdl_pose_desired_;
            KDL::Twist kdl_twist_desired_;

            /*! output 
             */
            KDL::Frame kdl_pose_measured_;
            KDL::Twist kdl_twist_measured_;
            KDL::Frame kdl_real_pose_measured_;

            KDL::JntArray kdl_current_joint_positions_;
            KDL::JntArrayVel kdl_current_joint_velocities_;
            KDL::JntArray kdl_desired_joint_positions_;

            /*! input to the controller for the nullspace optimization part
             */
            Eigen::VectorXd rest_posture_joint_configuration_;

        private:
            
            /*!
             * @return 
             */
            bool initMechanismChain();

            /*!
             * @return 
             */
            bool readParameters();

            /*!
             * @return 
             */
            bool initCartesianPidControllers();

            /*!
             * @return 
             */
            bool initNullspacePidControllers();

            /*!
             * @return 
             */
            bool initRTPublisher();

            /* ------------------------------
             * ! robot description 
             */
            hardware_interface::EffortJointInterface *hw_;
            urdf::Model robot_urdf_;

            /*!
             */
            ros::NodeHandle node_handle_;

            /*!
             */
            KDL::Twist kdl_twist_error_;
            KDL::Chain kdl_chain_;
            KDL::Jacobian kdl_chain_jacobian_;

            /*!
             */
            boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
            boost::scoped_ptr<KDL::ChainFkSolverVel> jnt_to_twist_solver_;
            boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

            /*!
             */
            double damping_;

            /*!
             */
            ros::Time last_time_;
            ros::Duration dt_;

            /*!
             */
            double ff_trans_;
            double ff_rot_;

            /*! feedback pid controllers (translation and rotation)
             */
            //std::vector<boost::shared_ptr<control_toolbox::Pid> > cartesian_fb_pid_controllers_;
            std::vector<control_toolbox::Pid> cartesian_fb_pid_controllers_;
            /*! feedback pid controllers (nullspace)
             */
            std::vector<boost::shared_ptr<control_toolbox::Pid> > nullspace_fb_pid_controllers_;
            
            /*!
             */
            std::vector<boost::shared_ptr<JointPositionController> >joint_position_controllers_;

            /*!
             */
            int num_joints_;

            Eigen::VectorXd eigen_desired_cartesian_velocities_;

            Eigen::VectorXd eigen_desired_joint_positions_;
            Eigen::VectorXd eigen_desired_joint_velocities_;

            Eigen::MatrixXd eigen_chain_jacobian_;

            Eigen::MatrixXd eigen_jac_times_jac_transpose_;
            Eigen::MatrixXd eigen_jjt_inverse_;
            Eigen::MatrixXd eigen_jac_pseudo_inverse_;
            Eigen::MatrixXd eigen_identity_;

            Eigen::VectorXd eigen_nullspace_term_;
            Eigen::MatrixXd eigen_nullspace_projector_;
            Eigen::VectorXd eigen_nullspace_error_;

            /*!
             */
            filters::MultiChannelTransferFunctionFilter<double> pose_filter_;
            std::vector<double> pose_unfiltered_data_;
            std::vector<double> pose_filtered_data_;

            /*!
             */
            int publisher_rate_;
            int publisher_counter_;
            int publisher_buffer_size_;
            int header_sequence_number_;

            /*
             * @param node_handle 
             * @oaram hw 
             * @param joint_position_controllers_
             * @return 
             */
            bool initJointPositionController(hardware_interface::EffortJointInterface *hw,
                                             ros::NodeHandle node_handle,
                                             std::vector<boost::shared_ptr<JointPositionController> >& joint_position_controllers);

            /*!
             */
            boost::shared_ptr<realtime_tools::RealtimePublisher<wam_dmp_controller::PoseTwistStamped> > pose_twist_desired_publisher_;
            boost::shared_ptr<realtime_tools::RealtimePublisher<wam_dmp_controller::PoseTwistStamped> > pose_twist_actual_publisher_;
            boost::shared_ptr<realtime_tools::RealtimePublisher<wam_dmp_controller::NullspaceTermStamped> > nullspace_term_publisher_;

            /*!
             */
            void publish();
    };
}
#endif
