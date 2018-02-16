#include <pluginlib/class_list_macros.h>
#include "wam_dmp_controller/euler_kinematical_rpy.h"
#include <angles/angles.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/LU>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include "wam_dmp_controller/opsrational_space_controller.h"
#include "wam_dmp_controller/effective_mass_matrix3d"

#define DEFAULT_KP_IM_LINK4 0.001
#define DEFAULT_KP_IM_LINK5 3
#define DEFAULT_KD_IM_LINK4 10
#define DEFAULT_KD_IM_LINK5 10

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

namespace wam_dmp_controllers
{
	OperationalSpaceController::OperationalSpaceController();
  	OperationalSpaceController::~OperationalSpaceController();

  	bool OperationalSpaceController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  	{
  		KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(hw, n);

  		// get use_simulation parameter from rosparam server
    	ros::NodeHandle nh;
    	nh.getParam("/use_simulation", use_simulation_);

    	// extend kdl chain with end-effector
    	extend_chain(n);

    	std::string root_seg_name;

    	nh_.getParam("root_seg_name", root_seg_name);

    	// instantiate solvers
    	// gravity_ is a member of KinematicChainControllerBase
    	dyn_param_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    	ee_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(extended_chain_));
    	wrist_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    	ee_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(extended_chain_));
    	ee_jacobian_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(extended_chain_));

    	// instantiate wrenches
    	wrench_wrist_ = KDL::Wrench();
    	base_wrench_wrist_ = KDL::Wrench();

    	// instantiate state and its derivatives
    	xyz_ws_ = Eigen::VectorXd(3);
    	xyz_dot_ws_ = Eigen::VectorXd(3);
    	xyz_des_ws_ = Eigen::VectorXd(3);
    	xyz_xdot_des_ws_ = Eigen::VectorXd(3);
    	vmax_ = Eigen::Vector3d(0);

    	// instantiate analytical to geometric transformation matrices
    	ws_E_ = Eigen::MatrixXd::Zero(6,6);
    	ws_E_.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3>::Identity();
    	ws_E_dot_ = Eigen::MatrixXd::Zero(6,6);

    	Kp_ = Eigen::MatrixXd::Zero(6, 6);
    	Kv_ = Eigen::MatrixXd::Zero(6, 6);
    	Ki_ = Eigen::MatrixXd::Zero(6, 6);
    	null_Kp_ = Eigen::MatrixXd::Zero(6, 6);
    	null_Kv_ = Eigen::MatrixXd::Zero(6, 6);
    	lamb_ = Eigen::MatrixXd::Zero(6, 6);
    	null_control_ = false;
              

    	for(int i = 0; i < 3; i++)
      		Kd_im_(i,i) = DEFAULT_KD_IM_LINK4;
    	for(int i = 3; i < 6; i++)
      		Kd_im_(i,i) = DEFAULT_KD_IM_LINK5;

      	// Start command subscriber 
        sub_command_ = n.subscribe<wam_dmp_controller::PoseRPY>("Cartesian_space_command", 1, &OperationalSpaceController::setCommandCB, this);

      	return true;
  	}

  	void OperationalSpaceController::starting(const ros::Time& time)
  	{

  		geometry_msgs::Vector3 xyz_command(0, 0, 0);
  		wam_dmp_controller::RPY rpy_command(0, 0, 0);
  		geometry_msgs::Vector3 dxyz_command(0, 0, 0);
  		wam_dmp_controller::RPY drpy_command(0, 0, 0);
  		command_struct_.xyz_ = xyz_command;
  		command_struct_.rpy_ = rpy_command;
  		command_struct_.dxyz_ = dxyz_command;
  		command_struct_.drpy_ = drpy_command;
  		command_.initRT(command_struct_);
  		// get current robot configuration (q and q dot)
    	for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
      	{
			joint_msr_states_.q(i) = joint_handles_[i].getPosition();
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
      	}

   		return true;
  	}

  	void update(const ros::Time& time, const ros::Duration& period)
  	{
  		//////////////////////////////////////////////////////////////////////////////////
    	//
    	// Robot configuration
    	//
    	//////////////////////////////////////////////////////////////////////////////////
    	//

    	// get current robot configuration (q and q dot)
    	for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
      	{
			joint_msr_states_.q(i) = joint_handles_[i].getPosition();
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
      	}

      	//
    	//////////////////////////////////////////////////////////////////////////////////
    
    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	// Joint Space Inertia Matrix M and Coriolis term C * q dot
    	// (solvers does not take into account anything past the wrist
    	//
    	//////////////////////////////////////////////////////////////////////////////////
    	//

      	// evaluate the current B(q)
    	KDL::JntSpaceInertiaMatrix M;
    	M.resize(kdl_chain_.getNrOfJoints());
    	dyn_param_solver_->JntToMass(joint_msr_states_.q, M);

    	// evaluate the current C(q) * q dot
    	KDL::JntArray C;
    	C.resize(kdl_chain_.getNrOfJoints());
    	dyn_param_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C);

    	/* libbarrett takes care of gravity compensation 
    	KDL::JntArray g;
    	g.resize(kdl_chain_.getNrOfJoints());
    	dyn_param_solver_->JntToCoriolis(joint_msr_states_.q, g);
		*/
    	//
    	//////////////////////////////////////////////////////////////////////////////////

    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	// Geometric Jacobians
    	//
    	//////////////////////////////////////////////////////////////////////////////////
    	//

    	KDL::Jacobian base_J_ee;
    	base_J_ee.resize(kdl_chain_.getNrOfJoints());
    	ee_jacobian_solver_->JntToJac(joint_msr_states_.q, base_J_ee);

    	/*
    	// evaluate the current geometric jacobian base_J_wrist
    	KDL::Jacobian base_J_wrist;
    	base_J_wrist.resize(kdl_chain_.getNrOfJoints());
    	wrist_jacobian_solver_->JntToJac(joint_msr_states_.q, base_J_wrist);
		*/
    	//
    	//////////////////////////////////////////////////////////////////////////////////

    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	// Forward Kinematics
    	//
    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	// end effector
    	KDL::Frame ee_fk_frame;
    	ee_fk_solver_->JntToCart(joint_msr_states_.q, ee_fk_frame);
    	//
    	//////////////////////////////////////////////////////////////////////////////////

    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	// Analytical Jacobian written w.r.t. the workspace frame ws_JA_ee
    	// ws_JA_ee = ws_TA * ws_J_ee
    	//
    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	// get the current ZYX representation PHI from R_ws_base * ee_fk_frame.M
    	double alpha, beta, gamma;
    	R_ws_ee_ = R_ws_base_ * ee_fk_frame.M;
    	p_ws_ee_ = p_ws_base + ee_fk_frame.p

    	R_ws_ee_.GetEulerZYX(alpha, beta, gamma);

    	// evaluate the transformation matrix between 
    	// the geometric and analytical jacobian TA
    	//
    	// ws_E = [eye(3), zeros(3);
    	//        zeros(3), inv(T(PHI))]
    	// where E is the Euler Kinematical Matrix
    	//
    	Eigen::Matrix3d E;
    	eul_kin_RPY(beta, alpha, E);
    	ws_E_.block<3,3>(3,3) = E.inverse();

    	// evaluate ws_J_ee
    	KDL::Jacobian ws_J_ee;
    	ws_J_ee.resize(kdl_chain_.getNrOfJoints());
    	KDL::changeBase(base_J_ee, R_ws_base_, ws_J_ee);

    	Eigen::MatrixXd ws_JA_ee;
    	ws_JA_ee = ws_E_ * ws_J_ee.data;
    	/////////////////////////////////////////////////////
    	for (uint32_t i = 0; i < 3; i++)
    	{
    		xyz_ws_[i] = p_ws_ee_[i];
    	}
    	xyz_dot_ws_ = ws_JA_ee * joint_msr_states_.qdot.data;
    	//////////////////////////////////////////////////////

    	 //////////////////////////////////////////////////////////////////////////////////
    	//
    	// Kinetic pseudo-energy Lamda (Siciliano page 297, operatoinal space dynamic model)
    	// (i.e. Operational Space Inertia Matrix)
    	//
    	//////////////////////////////////////////////////////////////////////////////////
    	//

    	Eigen::MatrixXd Lambda_inv;
    	Eigen::MatrixXd Lambda;
    	if(use_simulation_)
      		// use kdl matrix for simulation
      		Lambda_inv = ws_JA_ee * M.data.inverse() * ws_JA_ee.data.transpose();
    	else
      		// use kdl matrix for real scenario
      		Lambda_inv = ws_JA_ee * M.data.inverse() * ws_JA_ee.data.transpose();

    	EffectiveMassMatrix3d::compute(Lambda_inv, Lambda, 0.001);

    	//
    	//////////////////////////////////////////////////////////////////////////////////

    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	// Coriolis compensation in *Operational Space*
    	// Lambda * dot(ws_JA_ee) * qdot
    	//
    	//////////////////////////////////////////////////////////////////////////////////

    	// evaluation of dot(ws_JA_ee) = d/dt{ws_E_} * ws_J_ee + ws_E_ * d/dt{ws_J_ee}
    	//
    	// where d/dt{ws_TA} = [d/dt{eye(3)}, d/dt{zeros(3)}; 
    	//                      d/dt{zeros(3)}, d/dt{inv(T(PHI))}]
    	//                   = [zeros(3), zeros(3);
    	//                      zeros(3), -inv(T) * d/dt{T} * inv(T)]
    	//
    	// and d/dt{ws_J_ee} = [R_ws_base_, zeros(3);
    	//                      zeros(3), R_ws_base_] * d/dt{base_J_ee}
    	//
    	// evaluate the derivative of the state using the analytical jacobian
    	Eigen::Matrix3d ws_E_dot_;
    	ws_xdot_ = ws_JA_ee * joint_msr_states_.qd.data;
    	eul_kin_RPY_dot(beta, alpha, ws_xdot_(4), ws_xdot_(3), ws_E_dot_);
    	ws_E_dot_.block<3, 3>(3, 3) = -ws_E_.inverse(); * ws_E_dot_ * ws_E_.inverse();

    	// evaluate the derivative of the jacobian base_J_ee
    	KDL::JntArrayVel jnt_q_qdot;
    	KDL::Jacobian ws_J_ee_dot;
    	ws_J_ee_dot.reisze(kdl_chain_.getNrOfJoints());
    	jnt_q_qdot.q = joint_msr_states_.q;
    	jnt_q_qdot.dq = joint_msr_states_.qdot;
    	ee_jacobian_dot_solver_->JntToJacDot(jnt_q_qdot, ws_J_ee_dot);

    	// and project it in the workspace frame
    	ws_J_ee_dot.changeBase(R_ws_base_);
    	Eigen::MatrixXd ws_JA_ee_dot;
    	ws_JA_ee_dot = ws_E_dot_ * ws_J_ee.data + ws_E_ * ws_J_ee_dot.data;

    	Eigen::MatrixXd ws_JA_ee_dot;

    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	// evaluate dynamics inversion command TAU
    	//
    	// inheriting controllers augment TAU by calling 
    	// set_command(desired_acceleration)
    	// so that TAU += command_filter * desired_accelration
    	//
    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	if(use_simulation_)
      		tau_ = C.data - ws_JA_ee.data.transpose() * Lambda * ws_JA_ee_dot * joint_msr_states_.qdot.data;
    	else
      		tau_ = C.data - ws_JA_ee.data.transpose() * Lambda * ws_JA_ee_dot * joint_msr_states_.qdot.data;
      	command_filter_ = ws_JA_ee.data.transpose() * Lambda;
    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	// 
    	// (see A Unified Approach for Motion and Force Control
    	// of Robot Manipulators: The Operational Space Formulation, Oussama Khatib
    	// for details on the definition of a dynamically consistent generalized inverse)
    	//
    	//////////////////////////////////////////////////////////////////////////////////
    	//  

    	// evaluate the generalized inver of the jacobian matrix
    	Eigen::MatrixXd J_dyn_inv;
    	if (use_simulation_)
    		J_dyn_inv = M.data.inverse() * ws_JA_ee.data.transpose() * Lambda;
    	else
    		J_dyn_inv = M.data.inverse() * ws_JA_ee.data.transpose() * Lambda;

    	

    	if (null_control_)
    	{
    		// evaluate the null space projection
    		Eigen::MatrixXd ns_projection = Eigen::Matrix<double, 7, 7>::Identity() - ws_JA_ee.data.transpose() * J_dyn_inv;
    		/* TODO add the secondary nullspace controller here */
    	}

    	/*
    	TODO ADD THE VELOCITY DAMPING HERE IF APPLICABLE
    	*/

    	tau_ += ws_JA_ee.data.transpose() * Lambda * (Kp_ * (xyz_ws_des_ - xyz_ws_) + Kv_ * (xyz_des_ws_ - xyz_xdot_des_ws_))

    	// set joint efforts
    	for(int i=0; i<kdl_chain_.getNrOfJoints(); i++)
      	{
			joint_handles_[i].setCommand(tau_(i));

			// required to exploit the JOINT IMPEDANCE MODE of the kuka manipulator
			joint_stiffness_handles_[i].setCommand(0);
			joint_damping_handles_[i].setCommand(0);
			joint_set_point_handles_[i].setCommand(joint_msr_states_.q(i));
      	}

  	}

  	void setCommand(geometry_msgs::Vector3 xyz)
  	{
  		command_struct_.xyz_ = xyz;
  		command_.writeFromNonRT(command_struct_);
  	}

  	void setCommand(geometry_msgs::Vector3 xyz, wam_dmp_controller::RPY rpy)
  	{
  		command_struct_.xyz_ = xyz;
  		command_struct_.dxyz_ = rpy;
  		command_.writeFromNonRT(command_struct_);
  	}

  	void setCommand(geometry_msgs::Vector3 xyz, wam_dmp_controller::RPY dxyz);
  	{
  		command_struct_.xyz_ = xyz;
  		command_struct_.dxyz_ = dxyz;
  		command_.writeFromNonRT(command_struct_);
  	}

  	void setCommand(geometry_msgs::Vector3 xyz, geometry_msgs::Vector3 dxyz, wam_dmp_controller::RPY rpy, wam_dmp_controller::drpy)
  	{
  		command_struct_.xyz_ = xyz;
  		command_struct_.dxyz_ = dxyz;
  		command_struct_.drpy_ = drpy;
  		command_struct_.dxyz_ = dxyz;
  		command_.writeFromNonRT(command_struct_);
  	}

  	// Start command subscriber 
    void OperationalSpaceController:setCommandCB(const wam_dmp_controller::PoseRPYConstPtr& msg)
    {
    	setCommand((msg->data).position, (msg->data).orientation);
    }


}


