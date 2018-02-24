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

//KDL include
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdljacdot/chainjnttojacdotsolver.hpp>

namespace wam_dmp_controller
{
 	class OperationalSpaceController : public wam_dmp_controller::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
 	{
  		public:
  			OperationalSpaceController();
  			~OperationalSpaceController();

  			bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  			void starting(const ros::Time& time);
  			void update(const ros::Time& time, const ros::Duration& period);

  			//void get_gains_inertia(double& kp_z, double& kp_gamma, double& kd_pos, double& kd_att);
    		//void set_gains_inertia(double kp_z, double kp_gamma, double kd_pos, double kd_att);
    		void set_p_wrist_ee(double x, double y, double z);
    		void set_p_base_ws(double x, double y, double z);
    		void set_ws_base_angles(double alpha, double beta, double gamma);
            //void setCommand(geometry_msgs::Vector3 xyz);
            //void setCommand(geometry_msgs::Vector3 xyz, wam_dmp_controller::RPY rpy);
            //void setCommand(geometry_msgs::Vector3 xyz, wam_dmp_controller::RPY dxyz);
            //void setCommand(geometry_msgs::Vector3 xyz, geometry_msgs::Vector3 dxyz, wam_dmp_controller::RPY rpy, wam_dmp_controller::drpy);
    		//void setCommand(double x, double y, double z, double raw, double yaw, double pitch);
            //void setCommand(double x, double y, double z, double roll, double yaw, doubl);
    		//void load_calib_data(double& total_mass, KDL::Vector& p_sensor_tool_com);

            /**
             * \brief Store position and velocity command in struct to allow easier realtime buffer usage 
            */
            struct Commands
            {
                Eigen::Vector3d trans_xyz_command_;
                Eigen::Vector3d rot_xyz_command_;
                Eigen::Vector3d trans_xyzdot_command_;
                Eigen::Vector3d rot_xyzdot_command_;
            };


    	private:
    		//void update_inertia_matrix(Eigen::MatrixXd& inertia_matrix);
    		//void extend_chain(ros::NodeHandle &n);

    		// params syntax
  			// x_J_y: Geometric Jacobian w.r.t reference point y expressed in the base frame x
  			// x_AJ_y: Analytic Jacobian w.r.t reference point y expressed in the base frame x
  			// R_x_y: Rotation from base frame y to base frame x
  			// x_wrench_y:  Wrench w.r.t reference point y expressed in the base frame x
  			// x_vector: vector expressed in basis x
    		// p_x_y: position vector from x to y
    		// ee: Reference point of interest (typically the tool tip)
    		// wrist: tip of the 7th link of the Barrett Wam
    		// E: matrix that mapps Geometrix Jacobian back to Analytic Jacobian
            
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
            Eigen::Vector3d trans_xyz_des_ws_, trans_xyzdot_des_ws_;
            Eigen::Vector3d rot_xyz_des_ws_, rot_xyzdot_des_ws_, trans_xyz_error_, rot_xyz_error_;
            Eigen::Matrix3d rot_xyz_des_ws_skew_;
            Eigen::Matrix3d rot_xyz_ws_mat_, rot_xyz_des_ws_mat_;
            Eigen::Quaternion<double> rot_xyz_error_qua_, rot_xyz_ws_qua_, rot_xyz_des_ws_qua_;
            Eigen::AngleAxisd Z_, Y_, X_, Z_des_, Y_des_, X_des_;
            double rot_vmax_, trans_vmax_;

  			KDL::Vector p_wrist_ee_;
    		KDL::Vector p_base_ws_;

    		// pointers to solvers
    		boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;
    		boost::scoped_ptr<KDL::ChainJntToJacSolver> ee_jacobian_solver_, wrist_jacobian_solver_;
    		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> ee_fk_solver_;
    		boost::scoped_ptr<KDL::ChainJntToJacDotSolver> ee_jacobian_dot_solver_;

    		// chain required to move the reference point of jacobians
    		KDL::Chain extended_chain_;

    		// these matrices are sparse and initialized in init()
    		Eigen::MatrixXd ws_E_, ws_E_dot_;

    		// null space controller gains
    		Eigen::Matrix<double, 6, 6> Kp_;
    		Eigen::Matrix<double, 6, 6> Kv_;
            Eigen::Matrix<double, 6, 6> Ki_;
            Eigen::Matrix<double, 6, 6> null_Kp_;
            Eigen::Matrix<double, 6 ,6> null_Kv_;
            Eigen::Matrix<double, 6, 6> lamb_;
            Eigen::MatrixXd J_dyn_inv;
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
            void setCommand(geometry_msgs::Vector3 position, wam_dmp_controller::RPY orientation);
            void setCommandCB(const wam_dmp_controller::PoseRPYConstPtr& msg);
  	};
} 
#endif
