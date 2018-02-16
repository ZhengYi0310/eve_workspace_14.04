#ifndef __WAM_DMP_CONTROLLER_OPERATIONAL_SPACE_CONTROLLER_H
#define __WAM_DMP_CONTROLLER_OPERATIONAL_SPACE_CONTROLLER_H

#include "wam_dmp_controller_controllers/KinematicChainControllerBase.h"

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

namespace wam_dmp_controllers
{
 	class CartesianInverseDynamicsController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
 	{
  		public:
  			OperationalSpaceController()
  			~OperationalSpaceController()

  			bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  			void starting(const ros::Time& time);
  			void update(const ros::Time& time, const ros::Duration& period);

  			void get_gains_inertia(double& kp_z, double& kp_gamma, double& kd_pos, double& kd_att);
    		void set_gains_inertia(double kp_z, double kp_gamma, double kd_pos, double kd_att);
    		void set_p_wrist_ee(double x, double y, double z);
    		void set_p_base_ws(double x, double y, double z);
    		void set_ws_base_angles(double alpha, double beta, double gamma);
        void setCommand(geometry_msgs::Vector3 xyz);
        void setCommand(geometry_msgs::Vector3 xyz, wam_dmp_controller::RPY rpy);
        void setCommand(geometry_msgs::Vector3 xyz, wam_dmp_controller::RPY dxyz);
        void setCommand(geometry_msgs::Vector3 xyz, geometry_msgs::Vector3 dxyz, wam_dmp_controller::RPY rpy, wam_dmp_controller::drpy);
    		void setCommand(double x, double y, double z, double raw, double yaw, double pitch);
        void setCommand(double x, double y, double z, double roll, double yaw, doubl);
    		void load_calib_data(double& total_mass, KDL::Vector& p_sensor_tool_com);

        /**
         * \brief Store position and velocity command in struct to allow easier realtime buffer usage 
         */
        struct Commands
        {
          geometry_msgs::Vector3 xyz_;
          geometry_msgs::Vector3 dxyz_;
          wam_dmp_controller::RPY rpy_;
          wam_dmp_controller::RPY drpy_;
        };


    	private:
    		//void update_inertia_matrix(Eigen::MatrixXd& inertia_matrix);
    		void extend_chain(ros::NodeHandle &n);

    		// params syntax
  			// x_J_y: Geometric Jacobian w.r.t reference point y expressed in the base frame x
  			// x_AJ_y: Analytic Jacobian w.r.t reference point y expressed in the base frame x
  			// R_x y: Rotation from base frame y to base frame x
  			// x_wrench_y:  Wrench w.r.t reference point y expressed in the base frame y
  			// x_vector: vector expressed in basis x
    		// p_x y: Arm from x to y
    		// ee: Reference point of interest (typically the tool tip)
    		// wrist: tip of the 7th link of the Barrett Wam
    		// E: matrix that mapps Geometrix Jacobian back to Analytic Jacobian

  			KDL::Rotation R_ws_base_;
  			KDL::Rotation R_ws_ee_;
        KDL::Vector p_ws_base_;
        KDL::Vector p_ws_ee_;
  			KDL::Wrench base_wrench_wrist_;
  			Eigen::Vectorxd xyz_ws_, xyz_dot_ws_;
        Eigen::Vectorxd xyz_des_ws_, xyz_xdot_des_ws_;
        Eigen::Vector3d vmax_;

  			KDL::Vector p_wrist_ee_;
    		KDL::Vector p_base_ws_;

    		// pointers to solvers
    		boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;
    		boost::scoped_ptr<KDL::ChainJntToJacSolver> ee_jacobian_solver_, wrist_jacobian_solver_;
    		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> ee_fk_solver_;
    		boost::scoped_ptr<KDL::ChainJntToJacDotSolver> ee_jacobian_dot_solver_;
    		//boost::scoped_ptr<KDL::ChainJntToJacSolver> im_link4_jacobian_solver_;
    		//boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> im_link4_fk_solver_;
    		//boost::scoped_ptr<KDL::ChainJntToJacSolver> im_link5_jacobian_solver_;
    		//boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> im_link5_fk_solver_;

    		// chain required to move the reference point of jacobians
    		KDL::Chain extended_chain_;
    		// chain required to control internal motion
    		//KDL::Chain im_link4_chain_;
    		//KDL::Chain im_link5_chain_;

    		// these matrices are sparse and initialized in init()
    		Eigen::MatrixXd ws_E_, ws_E_dot_;
    		//Eigen::MatrixXd base_E_im_link5_;

    		// null space controller gains
    		Eigen::Matrix<double, 6, 6> Kp_;
    		Eigen::Matrix<double, 6, 6> Kv_;
        Eigen::Matrix<double, 6, 6> Ki_;
        Eigen::Matrix<double, 6, 6> null_Kp_;
        Eigen::Matrix<double, 6 ,6> null_Kv_;
        Eigen::Matrix<double, 6, 6> lamb_;
        bool null_control_;
    		//double gamma_im_link5_initial_;

    		// commandss
    		Eigen::VectorXd tau_;
    		Eigen::MatrixXd command_filter_;
        realtime_tools::RealtimeBuffer<Commands> command_;
        Commands command_struct_; // pre-allocated member that is re-used to set the realtime buffer

    		ros::Subscriber sub_force_;
    		KDL::Wrench wrench_wrist_;

    		// use simulation flag
    		bool use_simulation_;

        ros::Subscriber sub_command_;
        void setCommandCB(const wam_dmp_controller::PoseRPY& msg);
  	};
} 
#endif