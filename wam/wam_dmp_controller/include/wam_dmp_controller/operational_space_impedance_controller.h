#ifndef __WAM_DMP_CONTROLLER_OPERATIONAL_SPACE_CONTROLLER_H
#define __WAM_DMP_CONTROLLER_OPERATIONAL_SPACE_CONTROLLER_H

#include "wam_dmp_controller/KinematicChainControllerBase.h"
#include <geometry_msgs/Vector3.h>
#include <wam_dmp_controller/RPY.h>
#include <wam_dmp_controller/skew_symmetric.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/WrenchStamped.h>
#include <wam_dmp_controller/RPY.h>
#include <wam_dmp_controller/PoseRPY.h>
#include <wam_dmp_controller/PoseRPYCommand.h>
#include <wam_dmp_controller/ImpedanceControllerGains.h>
#include <wam_dmp_controller/SISE_kalman_filter.hpp>

//KDL include
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdljacdot/chainjnttojacdotsolver.hpp>

#include <Eigen/Geometry>

namespace wam_dmp_controller
{
 	class OperationalSpaceImpedanceController : public wam_dmp_controller::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
 	{
        friend class OperationalSpaceImpedanceSplineController;
  		public:
  			OperationalSpaceImpedanceController();
  			~OperationalSpaceImpedanceController();

  			bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  			void starting(const ros::Time& time);
  			void update(const ros::Time& time, const ros::Duration& period);
            
            struct Commands
            {
                Eigen::Vector3d trans_xyz_command_;
                Eigen::Vector3d rot_xyz_command_;
                Eigen::Vector3d trans_xyzdot_command_;
                Eigen::Vector3d rot_xyzdot_command_;
                Eigen::Vector3d trans_xyzdotdot_command_;
                Eigen::Vector3d rot_xyzdotdot_command_;
            };


    	private:
            ros::ServiceServer set_cmd_gains_service_;
            ros::ServiceServer get_cmd_gains_service_;
            
            void set_cmd_traj_point(geometry_msgs::Vector3 position, wam_dmp_controller::RPY orientation);
            void set_cmd_traj_callback(const wam_dmp_controller::PoseRPYConstPtr& msg);
            bool set_cmd_gains(wam_dmp_controller::ImpedanceControllerGains::Request &req, 
                               wam_dmp_controller::ImpedanceControllerGains::Response &res);     
            bool get_cmd_gains(wam_dmp_controller::ImpedanceControllerGains::Request &req, 
                               wam_dmp_controller::ImpedanceControllerGains::Response &res);   
            void setCommandRT(Eigen::Vector3d trans_des, Eigen::Vector3d trans_dot_des, Eigen::Vector3d trans_dotdot_des, 
                              Eigen::Vector3d rot_des, Eigen::Vector3d rot_dot_des, Eigen::Vector3d rot_dotdot_des);
            void get_parameters(ros::NodeHandle &n);
            void get_estimator_parameters(ros::NodeHandle &n_estimator);
            void publish_info(const ros::Time& time);

            void set_p_sensor_cp(double x, double y, double z); // For hybrid force control
            void get_gains_null(Eigen::Matrix<double, 6, 6>& null_Kp, Eigen::Matrix<double, 6, 6>& null_Kv);
            void set_gains_null(Eigen::Matrix<double, 6, 6> null_Kp, Eigen::Matrix<double, 6, 6> null_Kv);
            void set_p_wrist_ee(double x, double y, double z);
            void set_p_base_ws(double x, double y, double z);
            void set_ws_base_angles(double alpha, double beta, double gamma);
            
            KDL::JntSpaceInertiaMatrix M;
            KDL::JntArray C;
            KDL::JntArray G;
  			KDL::Rotation R_ws_base_;
  			KDL::Rotation R_ws_ee_;
            KDL::Vector p_ws_base_;
            KDL::Vector p_ws_ee_;
  			KDL::Wrench base_wrench_wrist_;
    		KDL::Wrench wrench_wrist_;   
            Eigen::VectorXd ws_x_, ws_xdot_;
  			Eigen::Vector3d trans_xyz_ws_, trans_xyzdot_ws_;
            Eigen::Vector3d rot_xyz_ws_, rot_xyzdot_ws_;
            Eigen::Vector3d trans_xyz_des_ws_, trans_xyzdot_des_ws_, trans_xyzdotdot_des_ws_, trans_xyz_error_;
            Eigen::Vector3d rot_xyz_des_ws_, rot_xyzdot_des_ws_, rot_xyzdotdot_des_ws_, rot_xyz_error_;
            Eigen::Matrix3d rot_xyz_des_ws_skew_;
            Eigen::Matrix3d rot_xyz_ws_mat_, rot_xyz_des_ws_mat_;
            Eigen::Quaternion<double> rot_xyz_error_qua_, rot_xyz_ws_qua_, rot_xyz_des_ws_qua_;
            Eigen::AngleAxisd Z_, Y_, X_, Z_des_, Y_des_, X_des_;
            double rot_vmax_, trans_vmax_;

  			KDL::Vector p_wrist_ee_;
    		KDL::Vector p_base_ws_;
            KDL::Vector p_sensor_cp_;

    		// pointers to solvers
    		boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;
    		boost::scoped_ptr<KDL::ChainJntToJacSolver> ee_jacobian_solver_, wrist_jacobian_solver_;
    		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> ee_fk_solver_;
    		boost::scoped_ptr<KDL::ChainJntToJacDotSolver> ee_jacobian_dot_solver_;

    		// chain required to move the reference point of jacobians
    		KDL::Chain extended_chain_;

    		// these matrices are sparse and initialized in init()
    		Eigen::MatrixXd ws_E_, ws_E_dot_;

    		// null space PD controller gains
    		Eigen::Matrix<double, 6, 6> Kp_;
    		Eigen::Matrix<double, 6, 6> Kv_;
            Eigen::Matrix<double, 6, 6> Ki_;
            Eigen::Matrix<double, 6, 6> null_Kp_;
            Eigen::Matrix<double, 6 ,6> null_Kv_;
            Eigen::Matrix<double, 6, 6> lamb_;
            
            // desired mass matrix, damping and gains of the modeled mechanical system 
            Eigen::Matrix<double, 6, 6> Md_;
            Eigen::Matrix<double, 6, 6> Dd_;
            Eigen::Matrix<double, 6, 6> Pd_;
            Eigen::MatrixXd J_dyn_inv_;
            Eigen::MatrixXd J_dyn_;
            Eigen::Vector3d tau_trans_, tau_rot_, tau_null_;
    		Eigen::VectorXd tau_, F_unit_;
    		Eigen::MatrixXd command_filter_;
            Eigen::VectorXd q_rest_;
            //bool null_control_;
    		//double gamma_im_link5_initial_;

    		// commandss
            realtime_tools::RealtimeBuffer<Commands> command_buffer_;
            Commands command_struct_; // pre-allocated member that is re-used to set the realtime buffer

    		ros::Subscriber sub_force_;
    		// use simulation flag
    		bool use_simulation_;

            ros::Subscriber sub_command_;
            
            ros::Time last_publish_time_;
            double publish_rate_;

            /// For external force estimation/////////////////////////////////////////////////////////////////
            Eigen::MatrixXd A_, B_, C_, W_, V_, P_xx_0_, x0_, M_gain_, M_star_;
            Eigen::MatrixXd input_est_;
            KDL::Jacobian J_curr_, J_last_;
            double state_dim_;
            double input_dim_;
            double measurement_dim_;
            Eigen::VectorXd ext_f_last_;
            Eigen::VectorXd ext_f_curr_;
            Eigen::MatrixXd joint_acc_curr_; // From exponential smoothing
            Eigen::MatrixXd joint_acc_last_;
            Eigen::MatrixXd joint_torque_curr_;
            Eigen::MatrixXd joint_torque_last_;
            bool ext_f_est_;
            boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> > pub_ext_force_est_;
            boost::scoped_ptr<SISE_KalmanFilter> SISE_KalmanFilterPtr;
            /// For external force estimation////////////////////////////////////////////////////////////////
            

            boost::scoped_ptr<realtime_tools::RealtimePublisher<wam_dmp_controller::PoseRPY> > pub_cart_des_, pub_cart_dot_des_, pub_cart_dotdot_des_, pub_cart_err_;
  	};
} 
#endif
