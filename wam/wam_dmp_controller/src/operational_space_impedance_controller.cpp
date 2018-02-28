#include <pluginlib/class_list_macros.h>
#include "wam_dmp_controller/euler_kinematic_rpy.h"
#include <angles/angles.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include "wam_dmp_controller/operational_space_impedance_controller.h"
#include "wam_dmp_controller/effective_mass_matrix.hpp"

#define DEFAULT_KP_Trans 1200
#define DEFAULT_KP_Rot 1200
#define DEFAULT_KD_Trans 50
#define DEFAULT_KD_Rot 50
#define DEFAULT_P2P_TRAJ_DURATION 5.0
#define P2P_COEFF_3 10.0
#define P2P_COEFF_4 -15.0
#define P2P_COEFF_5 6.0


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

namespace wam_dmp_controller
{
	OperationalSpaceImpedanceController::OperationalSpaceImpedanceController() {};
  	OperationalSpaceImpedanceController::~OperationalSpaceImpedanceController() {};

  	bool OperationalSpaceImpedanceController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  	{
  		KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(hw, n);

        //extended_chain_(n);
        

        tau_.resize(6);
        M.resize(kdl_chain_.getNrOfJoints());
        C.resize(kdl_chain_.getNrOfJoints());
        G.resize(kdl_chain_.getNrOfJoints());
        command_struct_.trans_xyz_command_ = Eigen::Vector3d::Zero();
        command_struct_.rot_xyz_command_ = Eigen::Vector3d::Zero();
        command_struct_.trans_xyzdot_command_ = Eigen::Vector3d::Zero();
        command_struct_.rot_xyzdot_command_ = Eigen::Vector3d::Zero();
        command_struct_.trans_xyzdotdot_command_ = Eigen::Vector3d::Zero();
        command_struct_.rot_xyzdotdot_command_ = Eigen::Vector3d::Zero();
    	// instantiate solvers
    	dyn_param_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    	ee_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(extended_chain_));
    	wrist_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    	ee_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(extended_chain_));
    	ee_jacobian_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(extended_chain_));

    	// instantiate wrenches
        wrench_wrist_ = KDL::Wrench();
    	base_wrench_wrist_ = KDL::Wrench();



    	// instantiate state and its derivatives
        trans_xyz_ws_           = Eigen::Vector3d::Zero();
        trans_xyzdot_ws_        = Eigen::Vector3d::Zero();
        rot_xyz_ws_             = Eigen::Vector3d::Zero();
        rot_xyzdot_ws_          = Eigen::Vector3d::Zero();
        trans_xyz_des_ws_       = Eigen::Vector3d::Zero();
        trans_xyzdot_des_ws_    = Eigen::Vector3d::Zero();
        trans_xyzdotdot_des_ws_ = Eigen::Vector3d::Zero();
        trans_xyz_error_        = Eigen::Vector3d::Zero();
        rot_xyz_des_ws_         = Eigen::Vector3d::Zero();
        rot_xyzdot_des_ws_      = Eigen::Vector3d::Zero();
        rot_xyzdotdot_des_ws_   = Eigen::Vector3d::Zero();
        rot_xyz_error_          = Eigen::Vector3d::Zero();
        ws_x_                   = Eigen::VectorXd(6);
        ws_xdot_                = Eigen::VectorXd(6);
        tau_                    = Eigen::VectorXd(kdl_chain_.getNrOfJoints());
        q_rest_                 = Eigen::VectorXd(kdl_chain_.getNrOfJoints());
        command_filter_         = Eigen::MatrixXd::Zero(kdl_chain_.getNrOfJoints(), 6);
        F_unit_                 = Eigen::VectorXd(6);
        J_dyn_inv_              = Eigen::MatrixXd::Zero(6, 6);
        J_dyn_                  = Eigen::MatrixXd::Zero(6, 6);

        // For external force estimation 

        ext_f_last_ = Eigen::VectorXd(6);
        ext_f_last_.setZero();
        ext_f_curr_ = Eigen::VectorXd(6);
        ext_f_curr_.setZero();
        pub_ext_force_est_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(n, "ext_force_est", 100));
        SISE_KalmanFilterPtr.reset(new SISE_KalmanFilter());
        J_curr_.resize(kdl_chain_.getNrOfJoints());
        J_last_.resize(kdl_chain_.getNrOfJoints());
        joint_acc_curr_    = Eigen::MatrixXd::Zero(kdl_chain_.getNrOfJoints(), 1);
        joint_acc_last_    = Eigen::MatrixXd::Zero(kdl_chain_.getNrOfJoints(), 1);
        joint_torque_curr_ = Eigen::MatrixXd::Zero(kdl_chain_.getNrOfJoints(), 1);
        joint_torque_last_ = Eigen::MatrixXd::Zero(kdl_chain_.getNrOfJoints(), 1);
        input_est_         = Eigen::MatrixXd::Zero(kdl_chain_.getNrOfJoints(), 1);
    	// instantiate analytical to geometric transformation matrices
    	ws_E_                   = Eigen::MatrixXd::Zero(6,6);
    	ws_E_.block<3,3>(0,0)   = Eigen::Matrix<double, 3, 3>::Identity();
    	ws_E_dot_               = Eigen::MatrixXd::Zero(6,6);

    	null_Kp_                = Eigen::MatrixXd::Zero(6, 6);
    	null_Kv_                = Eigen::MatrixXd::Zero(6, 6);
    	lamb_                   = Eigen::MatrixXd::Zero(6, 6);
        /*
        p2p_traj_duration_ = DEFAULT_P2P_TRAJ_DURATION;
        p2p_traj_const_ = Eigen::MatrixXf(4, 6);
        */
        get_parameters(n);
        
        // Serivces 
        // advertise HybridImpedanceCommand service
        //set_cmd_traj_pos_service_ = n.advertiseService("set_traj_pos_cmd", &OperationalSpaceImpedanceController::set_cmd_traj_spline_srv, this); 
        //get_cmd_traj_pos_service_ = n.advertiseService("get_traj_pos_cmd", &OperationalSpaceImpedanceController::get_cmd_traj_spline_srv, this); 
      	// Start command subscriber 
        //
        set_cmd_gains_service_ = n.advertiseService("set_impedance_gains", 
                                                    &OperationalSpaceImpedanceController::set_cmd_gains, this);
        get_cmd_gains_service_ = n.advertiseService("get_impedance_gains", 
                                                    &OperationalSpaceImpedanceController::get_cmd_gains, this);
        sub_command_ = n.subscribe("Cartesian_space_command", 10, &OperationalSpaceImpedanceController::set_cmd_traj_callback, this);
  
        pub_cart_des_.reset(new realtime_tools::RealtimePublisher<wam_dmp_controller::PoseRPY>(n, "des_cart_pos", 100));
        pub_cart_dot_des_.reset(new realtime_tools::RealtimePublisher<wam_dmp_controller::PoseRPY>(n, "des_cart_vel", 100));
        pub_cart_dotdot_des_.reset(new realtime_tools::RealtimePublisher<wam_dmp_controller::PoseRPY>(n, "des_cart_acc", 100));
        pub_cart_err_.reset(new realtime_tools::RealtimePublisher<wam_dmp_controller::PoseRPY>(n, "cart_err", 100));

      	return true;
  	}

  	void OperationalSpaceImpedanceController::starting(const ros::Time& time)
  	{

  		command_buffer_.initRT(command_struct_);
        last_publish_time_ = time;
  		// get current robot configuration (q and q dot)
    	for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
      	{
			joint_msr_states_.q(i) = joint_handles_[i].getPosition();
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();   
      	}

        if (ext_f_est_)
        {
            // Initialize the SISE Kalman Filter 
            SISE_KalmanFilterPtr->InitializeLinearModel(A_, B_, C_, W_, V_);
            SISE_KalmanFilterPtr->InitializeEstimator(x0_, P_xx_0_);
            SISE_KalmanFilterPtr->updateCtrlMatrix(M_star_);
            KDL::Jacobian base_J_ee;
    	    base_J_ee.resize(kdl_chain_.getNrOfJoints());
    	    ee_jacobian_solver_->JntToJac(joint_msr_states_.q, base_J_ee);

    	    KDL::Frame ee_fk_frame;
    	    ee_fk_solver_->JntToCart(joint_msr_states_.q, ee_fk_frame);
    	
    	    // get the current ZYX representation PHI from R_ws_base * ee_fk_frame.M
    	    double alpha, beta, gamma;
    	    R_ws_ee_ = R_ws_base_ * ee_fk_frame.M;
    	    p_ws_ee_ = p_ws_base_ + ee_fk_frame.p;

    	    R_ws_ee_.GetEulerZYX(alpha, beta, gamma);

    	    // evaluate the transformation matrix between 
    	    // the geometric and analytical jacobian TA
    	    //
    	    // ws_E = [eye(3), zeros(3);
    	    //        zeros(3), inv(E(PHI))]
    	    // where E is the Euler Kinematical Matrix
    	    //
    	    Eigen::Matrix3d E;
            E = Eigen::Matrix3d::Identity();
    	    eul_kin_RPY(beta, alpha, E);
    	    ws_E_.block<3,3>(3,3) = E.inverse();

    	    // evaluate ws_J_ee
    	    KDL::Jacobian ws_J_ee;
    	    ws_J_ee.resize(kdl_chain_.getNrOfJoints());
    	    KDL::changeBase(base_J_ee, R_ws_base_, ws_J_ee);

    	    for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
      	    {
			    joint_torque_curr_(i) = joint_handles_[i].getEffort();
			    joint_torque_last_(i) = joint_handles_[i].getEffort();  
                joint_acc_curr_(i) = joint_acceleration_handles_[i].getPosition();
                joint_acc_last_(i) = joint_acceleration_handles_[i].getPosition();
      	    }
            J_curr_.data = ws_J_ee.data;
            J_last_.data = ws_J_ee.data;
        }
        //
        // set default trajectory (force and position)
        //set_default_traj();

   		return;
  	}

    void OperationalSpaceImpedanceController::update(const ros::Time& time, const ros::Duration& period)
  	{


        command_struct_ = *(command_buffer_.readFromRT());    
        /*
        for (uint32_t i = 0; i < 3; i++)
        {
            trans_xyz_des_ws_[i] = command_struct_.trans_xyz_command_[i];
            rot_xyz_des_ws_[i] = command_struct_.rot_xyz_command_[i];
            trans_xyzdot_des_ws_[i] = command_struct_.trans_xyzdot_command_[i];
            rot_xyzdot_des_ws_[i] = command_struct_.rot_xyzdot_command_[i];
        }
        */
        trans_xyz_des_ws_       = command_struct_.trans_xyz_command_;
        trans_xyzdot_des_ws_    = command_struct_.trans_xyzdot_command_;
        trans_xyzdotdot_des_ws_ = command_struct_.trans_xyzdotdot_command_;
        rot_xyz_des_ws_         = command_struct_.rot_xyz_command_;
        rot_xyzdot_des_ws_      = command_struct_.rot_xyzdot_command_;
        rot_xyzdotdot_des_ws_   = command_struct_.rot_xyzdotdot_command_;
        ////////////////////////////// 
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

      	// evaluate the current M(q)
    	dyn_param_solver_->JntToMass(joint_msr_states_.q, M);

    	// evaluate the current C(q) * q dot
    	dyn_param_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C);

        // gravity compensation
        dyn_param_solver_->JntToGravity(joint_msr_states_.q, G);
        ////////////////////////////////////////////////////////////
        if (ext_f_est_)
        {
    	    for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
      	    {
			    joint_torque_curr_(i) = joint_handles_[i].getEffort();
                joint_acc_curr_(i) = joint_acceleration_handles_[i].getPosition();
      	    }
            
            SISE_KalmanFilterPtr->estimate(joint_torque_curr_, joint_torque_last_);
            input_est_ = SISE_KalmanFilterPtr->getInput()->getMean().block(0, 0, state_dim_ ,1);
            input_est_ -= joint_acc_curr_;
            input_est_ += joint_acc_last_;
            input_est_ = M_star_ * input_est_;
            input_est_ += J_last_.data.transpose() * ext_f_last_; 
        }
        ////////////////////////////////////////////////////////////

    	//////////////////////////////////////////////////////////////////////////////////

    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	// Geometric Jacobians
    	//
    	//////////////////////////////////////////////////////////////////////////////////

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
    	p_ws_ee_ = p_ws_base_ + ee_fk_frame.p;

    	R_ws_ee_.GetEulerZYX(alpha, beta, gamma);

    	// evaluate the transformation matrix between 
    	// the geometric and analytical jacobian TA
    	//
    	// ws_E = [eye(3), zeros(3);
    	//        zeros(3), inv(E(PHI))]
    	// where E is the Euler Kinematical Matrix
    	//
    	Eigen::Matrix3d E;
        E = Eigen::Matrix3d::Identity();
    	eul_kin_RPY(beta, alpha, E);
    	ws_E_.block<3,3>(3,3) = E.inverse();

    	// evaluate ws_J_ee
    	KDL::Jacobian ws_J_ee;
    	ws_J_ee.resize(kdl_chain_.getNrOfJoints());
    	KDL::changeBase(base_J_ee, R_ws_base_, ws_J_ee);

    	Eigen::MatrixXd ws_JA_ee;
    	ws_JA_ee = ws_E_ * ws_J_ee.data;
    	/////////////////////////////////////////////////////
        trans_xyz_ws_ << p_ws_ee_.x(), p_ws_ee_.y(), p_ws_ee_.z();
        rot_xyz_ws_ << gamma, beta, alpha;
    	trans_xyzdot_ws_ = (ws_JA_ee * joint_msr_states_.qdot.data).block(0, 0, 3, 1);
        rot_xyzdot_ws_ = (ws_JA_ee * joint_msr_states_.qdot.data).block(3, 0, 3, 1); 
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
      		//Lambda_inv = ws_JA_ee * M.data.inverse() * ws_JA_ee.transpose();
            // use the Geometric Jacobian here 
            Lambda_inv = ws_JA_ee * M.data.inverse() * ws_J_ee.data.transpose();
    	else
      		// use kdl matrix for real scenario
      		//Lambda_inv = ws_JA_ee * M.data.inverse() * ws_JA_ee.transpose();
            Lambda_inv = ws_JA_ee * M.data.inverse() * ws_J_ee.data.transpose();

    	ComputeMassMatrix(Lambda_inv, Lambda);

        // Always consider null-space control, otherwise might cause unstable behavior for redundant Manipulator 
        // in joint Space
        // use the Geometric Jacobian here 
    	if(use_simulation_)
      		// use kdl matrix for simulation
      		//Lambda_inv = ws_JA_ee * M.data.inverse() * ws_JA_ee.transpose();
            // use the Geometric Jacobian here 
            J_dyn_ = ws_J_ee.data * M.data.inverse() * ws_J_ee.data.transpose();
    	else
      		// use kdl matrix for real scenario
      		//Lambda_inv = ws_JA_ee * M.data.inverse() * ws_JA_ee.transpose();
            J_dyn_ = ws_J_ee.data * M.data.inverse() * ws_J_ee.data.transpose();

    	ComputeMassMatrix(J_dyn_, J_dyn_inv_);
        
        J_dyn_inv_ = M.data.inverse() * ws_J_ee.data.transpose() * J_dyn_inv_;

        /////////////////////////////
        if (ext_f_est_)
        {
            ext_f_curr_ = J_dyn_inv_.transpose() * input_est_;
            J_last_.data = ws_J_ee.data;
        } 
        /////////////////////////////
    	//////////////////////////////////////////////////////////////////////////////////

    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	// Coriolis compensation in *Operational Space*
    	// Lambda * dot(ws_JA_ee) * qdot
    	//
    	//////////////////////////////////////////////////////////////////////////////////

    	// evaluation of dot(ws_JA_ee) = d/dt{ws_E_} * ws_J_ee + ws_E_ * d/dt{ws_J_ee}
    	//
    	// where d/dt{ws_E_} = [d/dt{eye(3)}, d/dt{zeros(3)}; 
    	//                      d/dt{zeros(3)}, d/dt{inv(ws_E_(PHI))}]
    	//                   = [zeros(3), zeros(3);
    	//                      zeros(3), -inv(ws_E_) * d/dt{ws_E_} * inv(ws_E_)]
    	//
    	// and d/dt{ws_J_ee} = [R_ws_base_, zeros(3);
    	//                      zeros(3), R_ws_base_] * d/dt{base_J_ee}
    	//
    	// evaluate the derivative of the state using the analytical jacobian
    	Eigen::Matrix3d E_dot_;
        E_dot_ = Eigen::Matrix3d::Identity();
    	ws_xdot_ = ws_JA_ee * joint_msr_states_.qdot.data;
    	eul_kin_RPY_dot(beta, alpha, ws_xdot_(4), ws_xdot_(3), E_dot_);
    	ws_E_dot_.block<3, 3>(3, 3) = -E.inverse() * E_dot_ * E.inverse();

    	// evaluate the derivative of the jacobian base_J_ee
    	KDL::JntArrayVel jnt_q_qdot;
    	KDL::Jacobian ws_J_ee_dot;
    	ws_J_ee_dot.resize(kdl_chain_.getNrOfJoints());
    	jnt_q_qdot.q = joint_msr_states_.q;
    	jnt_q_qdot.qdot = joint_msr_states_.qdot;
    	ee_jacobian_dot_solver_->JntToJacDot(jnt_q_qdot, ws_J_ee_dot);

    	// and project it in the workspace frame
    	ws_J_ee_dot.changeBase(R_ws_base_);
    	Eigen::MatrixXd ws_JA_ee_dot;
        ws_JA_ee_dot = ws_E_dot_ * ws_J_ee.data + ws_E_ * ws_J_ee_dot.data;

    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	// evaluate dynamics inversion command TAU
    	//
    	// inheriting controllers augment TAU by calling 
    	// set_command(desired_acceleration)
    	// so that TAU += command_filter * desired_accelration
    	//
    	//////////////////////////////////////////////////////////////////////////////////
    	// use Geometric Jaocbian here 
      	//command_filter_ = ws_JA_ee.transpose() * Lambda;
        command_filter_ = ws_J_ee.data.transpose() * Lambda;
    	if(use_simulation_)
      		tau_ = C.data + G.data - command_filter_ * ws_JA_ee_dot * joint_msr_states_.qdot.data;
    	else  // gravity handled by the hardware interface itself when not using simulation 
      		tau_ = C.data - command_filter_ * ws_JA_ee_dot * joint_msr_states_.qdot.data;
        /////////////////////////////////////////////
        if (ext_f_est_)
        {
            tau_ += input_est_;
        }
        /////////////////////////////////////////////
    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	// 
    	// (see A Unified Approach for Motion and Force Control
    	// of Robot Manipulators: The Operational Space Formulation, Oussama Khatib
    	// for details on the definition of a dynamically consistent generalized inverse)
    	//
    	//////////////////////////////////////////////////////////////////////////////////
        /* An easier approch based on RPY angles and no clipping 
         * 
         * tau_trans_ = trans_xyzdotdot_des_ws_ + Kp_.block(0, 0, 3, 3) * (trans_xyz_des_ws_ - trans_xyz_ws_) + Kv_.block(0, 0, 3, 3) * (trans_xyzdot_des_ws_ - trans_xyzdot_ws_);
         * rot_xyz_error_ = rot_xyz_des_ws_ - rot_xyz_ws_;
         * for (int i = 0; i < 3; i++)
         * {
         *      rot_xyz_error_(i) = angles::normalize_angle(rot_xyz_error_(i)); 
         * }
         * tau_rot_ = rot_xyzdotdot_des_ws_ + Kp_.block(3, 3, 3, 3) * rot_xyz_error_ + Kv_.block(3, 3, 3, 3) * (rot_xyzdot_des_ws_ - rot_xyzdot_ws_);
         *  F_unit_.block(0, 0, 3, 1) = tau_trans_;
            F_unit_.block(3, 0, 3, 1) = tau_rot_;
         * 
         *
         */
        trans_xyz_error_ = Kp_.block(0, 0, 3, 3) * (trans_xyz_des_ws_ - trans_xyz_ws_);
        if(trans_xyz_error_.norm() > trans_vmax_)
        {
            trans_xyz_error_ = trans_vmax_ / trans_xyz_error_.norm() * trans_xyz_error_;
        }

        tau_trans_ = Kv_.block(0, 0, 3, 3) * (trans_xyzdot_ws_ - trans_xyz_error_);
        
        Z_ = Eigen::AngleAxisd(rot_xyz_ws_[2], Eigen::Vector3d::UnitZ());
        Y_ = Eigen::AngleAxisd(rot_xyz_ws_[1], Eigen::Vector3d::UnitY());
        X_ = Eigen::AngleAxisd(rot_xyz_ws_[0], Eigen::Vector3d::UnitX());
        Z_des_ = Eigen::AngleAxisd(rot_xyz_des_ws_[2], Eigen::Vector3d::UnitZ());
        Y_des_ = Eigen::AngleAxisd(rot_xyz_des_ws_[1], Eigen::Vector3d::UnitY());
        X_des_ = Eigen::AngleAxisd(rot_xyz_des_ws_[0], Eigen::Vector3d::UnitX());
        

        rot_xyz_ws_qua_ = rot_xyz_ws_mat_ = Z_ * Y_ * X_;
        rot_xyz_des_ws_qua_ = rot_xyz_des_ws_mat_ = Z_des_ * Y_des_ * X_des_;
        rot_xyz_error_qua_ = rot_xyz_ws_mat_ * rot_xyz_des_ws_mat_.inverse();
        
        /* 
        // Opeational Space Control: A Theoretic and Empirical Comparison 
        KDL::Vector temp = KDL::Vector(rot_xyz_des_ws_qua_.x(), rot_xyz_des_ws_qua_.y(), rot_xyz_des_ws_qua_.z());
        skew_symmetric(temp, rot_xyz_des_ws_skew_);
        rot_xyz_error_ = rot_xyz_des_ws_qua_.w() * rot_xyz_ws_qua_.vec() - rot_xyz_ws_qua_.w() * rot_xyz_des_ws_qua_.vec() + rot_xyz_des_ws_skew_ *  rot_xyz_ws_qua_.vec();
        tau_rot_ =  Kv_.block(3, 3, 3, 3) * (rot_xyzdot_ws_ - Kp_.block(3, 3, 3, 3) * rot_xyz_error_); 
        */
        double norm = sqrt(rot_xyz_error_qua_.x() * rot_xyz_error_qua_.x() + rot_xyz_error_qua_.y() * rot_xyz_error_qua_.y() + rot_xyz_error_qua_.z() * rot_xyz_error_qua_.z());
        double c = 0.0;
        if(norm != 0.0)
        {
            c = 2.0 * acos(rot_xyz_error_qua_.w()) / norm;
            
            if(c > rot_vmax_)
                c = rot_vmax_;
            else if(c < -rot_vmax_)
                c = -rot_vmax_;
        }
        rot_xyz_error_ << rot_xyz_error_qua_.x() * c, rot_xyz_error_qua_.y() * c, rot_xyz_error_qua_.z() * c;
        tau_rot_ = Kv_.block(3, 3, 3, 3) * (rot_xyzdot_ws_ + Kp_.block(3, 3, 3, 3) * rot_xyz_error_);
        F_unit_.block(0, 0, 3, 1) = tau_trans_;
        F_unit_.block(3, 0, 3, 1) = tau_rot_;
        tau_ += command_filter_ * F_unit_;

        // Consider the nullspace controller here 
        tau_null_ = M.data * (null_Kp_ * (q_rest_ - joint_msr_states_.q.data) - null_Kv_ * joint_msr_states_.qdot.data);
        tau_ += tau_null_;

        //
        // evaluate position vector between the origin of the workspace frame and the end-effector
        KDL::Vector p_ws_ee;
        p_ws_ee = R_ws_base_ * (ee_fk_frame.p - p_base_ws_);
    

        // evaluate the state
        ws_x_ << p_ws_ee(0), p_ws_ee(1), p_ws_ee(2), gamma, beta, alpha;
    	// set joint efforts
    	for(int i=0; i<kdl_chain_.getNrOfJoints(); i++)
      	{
			joint_handles_[i].setCommand(tau_(i));

			// required to exploit the JOINT IMPEDANCE MODE of the kuka manipulator
			joint_stiffness_handles_[i].setCommand(0);
			joint_damping_handles_[i].setCommand(0);
			joint_set_point_handles_[i].setCommand(joint_msr_states_.q(i));
      	}

        if (time > last_publish_time_ + ros::Duration(1.0 / publish_rate_))
        {
            last_publish_time_ += ros::Duration(1.0 / publish_rate_);
            publish_info(time);
        }

        if (ext_f_est_)
        {
            joint_acc_last_ = joint_acc_curr_;
            joint_torque_last_ = joint_torque_curr_;            
            ext_f_last_ = ext_f_curr_;
        }
    }

    void OperationalSpaceImpedanceController::publish_info(const ros::Time& time)
    {
        if (pub_cart_des_ && pub_cart_des_->trylock())
        {
            pub_cart_des_->msg_.stamp = time;
            pub_cart_des_->msg_.position.x = trans_xyz_des_ws_[0];
            pub_cart_des_->msg_.position.y = trans_xyz_des_ws_[1];
            pub_cart_des_->msg_.position.z = trans_xyz_des_ws_[2];
            pub_cart_des_->msg_.orientation.roll = rot_xyz_des_ws_[0];
            pub_cart_des_->msg_.orientation.pitch = rot_xyz_des_ws_[1];
            pub_cart_des_->msg_.orientation.yaw = rot_xyz_des_ws_[2];
        }

        if (pub_cart_dot_des_ && pub_cart_dot_des_->trylock())
        {
            pub_cart_dot_des_->msg_.stamp = time;
            pub_cart_dot_des_->msg_.position.x = trans_xyzdot_des_ws_[0];
            pub_cart_dot_des_->msg_.position.y = trans_xyzdot_des_ws_[1];
            pub_cart_dot_des_->msg_.position.z = trans_xyzdot_des_ws_[2];
            pub_cart_dot_des_->msg_.orientation.roll = rot_xyzdot_des_ws_[0];
            pub_cart_dot_des_->msg_.orientation.pitch = rot_xyzdot_des_ws_[1];
            pub_cart_dot_des_->msg_.orientation.yaw = rot_xyzdot_des_ws_[2];
        }

        /* get the desired accel
        if (pub_cart_dotdot_des_ && pub_cart_dotdot_des_->trylock())
        {
            pub_cart_dotdot_des_->msg_.stamp = time;
            pub_cart_dotdot_des_->msg_.position.x = trans_xyzdot_des_ws_[0];
            pub_cart_dotdot_des_->msg_.position.y = trans_xyzdot_des_ws_[1];
            pub_cart_dotdot_des_->msg_.position.z = trans_xyzdot_des_ws_[2];
            pub_cart_dotdot_des_->msg_.orientation.roll = rot_xyzdot_des_ws_[0];
            pub_cart_dotdot_des_->msg_.orientation.pitch = rot_xyzdot_des_ws_[1];
            pub_cart_dotdot_des_->msg_.orientation.yaw = rot_xyzdot_des_ws_[2];
        }
        */
        if (pub_cart_err_ && pub_cart_err_->trylock())
        {
            pub_cart_err_->msg_.stamp = time;
            pub_cart_err_->msg_.position.x = trans_xyz_error_[0];
            pub_cart_err_->msg_.position.y = trans_xyz_error_[1];
            pub_cart_err_->msg_.position.z = trans_xyz_error_[2];
            pub_cart_err_->msg_.orientation.roll = rot_xyz_error_[0];
            pub_cart_err_->msg_.orientation.pitch = rot_xyz_error_[1];
            pub_cart_err_->msg_.orientation.yaw = rot_xyz_error_[2];
        }

        /* TODO publish the estimated external force 
         */
        pub_cart_des_->unlockAndPublish();
        pub_cart_dot_des_->unlockAndPublish();
        //pub_cart_dotdot_des_->unlockAndPublish();
        pub_cart_err_->unlockAndPublish();

        if (ext_f_est_)
        {
            if (pub_ext_force_est_ && pub_ext_force_est_->trylock())
            {
                pub_ext_force_est_->msg_.header.stamp = time;
                pub_ext_force_est_->msg_.wrench.force.x = ext_f_curr_[0];
                pub_ext_force_est_->msg_.wrench.force.y = ext_f_curr_[1];
                pub_ext_force_est_->msg_.wrench.force.z = ext_f_curr_[2];
                pub_ext_force_est_->msg_.wrench.torque.x = ext_f_curr_[3];
                pub_ext_force_est_->msg_.wrench.torque.y = ext_f_curr_[4];
                pub_ext_force_est_->msg_.wrench.torque.z = ext_f_curr_[5];
            }
            pub_ext_force_est_->unlockAndPublish();
        }
        //pub_ext_force_est_->unlockAndPublish;
    }
    
    void OperationalSpaceImpedanceController::get_parameters(ros::NodeHandle &n)
    {
        std::vector<std::string> name;
        name.push_back("trans_x");
        name.push_back("trans_y");
        name.push_back("trans_z");
        name.push_back("rot_x");
        name.push_back("rot_y");
        name.push_back("rot_z");
        ros::NodeHandle Kp_handle(n, "Kp_Gains");
        ros::NodeHandle Kv_handle(n, "Kv_Gains");
        ros::NodeHandle Ki_handle(n, "Ki_Gains");
        ros::NodeHandle null_Kp_handle(n, "null_Kp_gains");
        ros::NodeHandle null_Kv_handle(n, "null_Kv_gains");
        ros::NodeHandle rest_postures_handle(n, "rest_posture");
         
        for (size_t i = 0; i < 6; i++)
        {
            if ( !Kp_handle.getParam(name[i].c_str(), Kp_.coeffRef(i, i))) 
            {
                ROS_WARN("Kp gain not set for %s in yaml file, Using 300.", name[i].c_str());
                Kp_.coeffRef(i, i) = 300;
            }
            else 
            {
                ROS_WARN("Kp gain set for %s in yaml file, Using %f", name[i].c_str(), Kp_.coeff(i, i)); 
            }
            
            if ( !Kv_handle.getParam(name[i].c_str(), Kv_.coeffRef(i, i))) 
            {
                ROS_WARN("Kv gain not set for %s in yaml file, Using 300.", name[i].c_str());
                Kv_.coeffRef(i, i) = 300;
            }
            else 
            {
                ROS_WARN("Kv gain set for %s in yaml file, Using %f", name[i].c_str(), Kv_.coeff(i, i)); 
            }

            if ( !Ki_handle.getParam(name[i].c_str(), Ki_.coeffRef(i, i))) 
            {
                ROS_WARN("Ki gain not set for %s in yaml file, Using 1.", name[i].c_str());
                Ki_.coeffRef(i, i) = 1;
            }
            else 
            {
                ROS_WARN("Ki gain set for %s in yaml file, Using %f", name[i].c_str(), Ki_.coeff(i, i)); 
            }

            if ( !null_Kp_handle.getParam(name[i].c_str(), null_Kp_.coeffRef(i, i))) 
            {
                ROS_WARN("null_Kp gain not set for %s in yaml file, Using 0.", name[i].c_str());
                null_Kp_.coeffRef(i, i) = 0;
            }
            else 
            {
                ROS_WARN("null_Kp gain set for %s in yaml file, Using %f", name[i].c_str(), null_Kp_.coeff(i, i)); 
            }
            
            if ( !null_Kv_handle.getParam(name[i].c_str(), null_Kv_.coeffRef(i, i))) 
            {
                ROS_WARN("null_Kv gain not set for %s in yaml file, Using 0.", name[i].c_str());
                null_Kv_.coeffRef(i, i) = 0;
            }
            else 
            {
                ROS_WARN("null_Kv gain set for %s in yaml file, Using %f", name[i].c_str(), null_Kv_.coeff(i, i)); 
            }
        }
        for (size_t i = 0; i < joint_handles_.size(); i++)
        {
            //TODO also need to load the rest joint postures from the parameter server.....
            if ( !rest_postures_handle.getParam(joint_handles_[i].getName().c_str(), q_rest_[i])) 
            {
                ROS_WARN("rest posture not set for %s in yaml file, Using 0.", joint_handles_[i].getName().c_str());
                q_rest_[i] = 0.0;
            }
            else
            {
                ROS_WARN("rest_posture set for %s in yaml file, Using %f",  joint_handles_[i].getName().c_str(), q_rest_[i]); 
            }
        }

        // vector from the wrist to the tool tip (projected in wrist_palm frame)
        std::vector<double> p_wrist_ee;
        // vector from the sensor to the contact point
        std::vector<double> p_sensor_cp; // only for hybrid motion/force control
        // vector from vito_anchor to the workspace (projected in world frame)
        std::vector<double> p_base_ws;
        // rotation of the workspace frame w.r.t. vito_anchor frame
        std::vector<double> ws_base_angles;
        
        n.getParam("p_wrist_ee", p_wrist_ee);
        n.getParam("p_sensor_cp", p_sensor_cp);
        n.getParam("p_base_ws", p_base_ws);
        n.getParam("ws_base_angles", ws_base_angles);

        // set internal members
        set_p_wrist_ee(p_wrist_ee.at(0), p_wrist_ee.at(1), p_wrist_ee.at(2));
        set_p_sensor_cp(p_sensor_cp.at(0), p_sensor_cp.at(1), p_sensor_cp.at(2));
        set_p_base_ws(p_base_ws.at(0), p_base_ws.at(1), p_base_ws.at(2));
        set_ws_base_angles(ws_base_angles.at(0), ws_base_angles.at(1), ws_base_angles.at(2));
        
        n.getParam("publish_rate", publish_rate_);
        n.getParam("use_simulation_", use_simulation_);
        n.getParam("external_force_estimation_", ext_f_est_); // Whetherto do external force estimation.

        if (ext_f_est_)
        {
            ros::NodeHandle SISE_handle(n, "SISE");
            get_estimator_parameters(SISE_handle);
        }
    }

    void OperationalSpaceImpedanceController::get_estimator_parameters(ros::NodeHandle &n_estimators)
    {       
        if (n_estimators.getParam("state_dim", state_dim_))
        {
            ROS_ERROR("state dimension not specified!");
            return;
        }
        else 
        {
            ROS_INFO("state dimension is set to %f", state_dim_);
        }

        if (n_estimators.getParam("input_dim", input_dim_))
        {
            ROS_ERROR("input dimension not specified!");
            return;             
        }
        else 
        {
            ROS_INFO("input dimension is set to %f", state_dim_);
        }

        if (n_estimators.getParam("measurement_dim", measurement_dim_))
        {
            ROS_ERROR("measurement dimension not specified!");
            return;             
        }
        else 
        {
            ROS_INFO("measurement dimension is set to %f", measurement_dim_);
        }
        A_      = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
        B_      = Eigen::MatrixXd::Zero(state_dim_, input_dim_);
        C_      = Eigen::MatrixXd::Zero(measurement_dim_, state_dim_);
        W_      = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
        V_      = Eigen::MatrixXd::Zero(measurement_dim_, measurement_dim_);
        P_xx_0_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
        x0_     = Eigen::MatrixXd::Zero(state_dim_, 1);
        M_gain_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);

        std::vector<double> A_vec;
        std::vector<double> B_vec;
        std::vector<double> C_vec;
        std::vector<double> W_vec;
        std::vector<double> V_vec;
        std::vector<double> Pxx0_vec;
        std::vector<double> x0_vec;
        std::vector<double> M_gain_vec;

        n_estimators.getParam("A", A_vec);
        n_estimators.getParam("B", B_vec);
        n_estimators.getParam("C", C_vec);
        n_estimators.getParam("W", W_vec);
        n_estimators.getParam("V", V_vec);
        n_estimators.getParam("P_xx_0", Pxx0_vec);
        n_estimators.getParam("x0", x0_vec);
        n_estimators.getParam("M_gain_", M_gain_vec);
        
        for (int i = 0; i < A_vec.size(); i++)
        {
            A_ << A_vec[i];
        }
        
        for (int i = 0; i < B_vec.size(); i++)
        {
            B_ << B_vec[i];
        }
        for (int i = 0; i < C_vec.size(); i++)
        {
            C_ << C_vec[i];
        }
        for (int i = 0; i < x0_vec.size(); i++)
        {
            x0_ << x0_vec[i];
        }
        
        
        W_      = Eigen::Map<Eigen::VectorXd>(&W_vec[0], W_vec.size()).asDiagonal();
        V_      = Eigen::Map<Eigen::VectorXd>(&V_vec[0], V_vec.size()).asDiagonal();
        P_xx_0_ = Eigen::Map<Eigen::VectorXd>(&Pxx0_vec[0], Pxx0_vec.size()).asDiagonal();
        M_gain_ = Eigen::Map<Eigen::VectorXd>(&M_gain_vec[0], M_gain_vec.size()).asDiagonal();

        M_star_ << M_gain_, Eigen::MatrixXd::Ones(state_dim_, state_dim_);
    }
    
    
    void OperationalSpaceImpedanceController::set_cmd_traj_point(geometry_msgs::Vector3 position, wam_dmp_controller::RPY orientation)
    {
        command_struct_.trans_xyz_command_[0] = position.x;
        command_struct_.trans_xyz_command_[1] = position.y;
        command_struct_.trans_xyz_command_[2] = position.z;
        command_struct_.rot_xyz_command_[0] = orientation.roll;
        command_struct_.rot_xyz_command_[1]= orientation.pitch;
        command_struct_.rot_xyz_command_[2] = orientation.yaw;

        command_struct_.trans_xyzdot_command_ = Eigen::Vector3d::Zero();
        command_struct_.rot_xyzdot_command_ = Eigen::Vector3d::Zero();
        command_struct_.trans_xyzdotdot_command_ = Eigen::Vector3d::Zero();
        command_struct_.rot_xyzdotdot_command_ = Eigen::Vector3d::Zero();
        
        //run_spline_ = false;
        command_buffer_.writeFromNonRT(command_struct_);  
    }
  	// Start command subscriber 
    void OperationalSpaceImpedanceController::set_cmd_traj_callback(const wam_dmp_controller::PoseRPYConstPtr& msg)
    {
        set_cmd_traj_point(msg->position, msg->orientation);

    }

    bool OperationalSpaceImpedanceController::set_cmd_gains(wam_dmp_controller::ImpedanceControllerGains::Request &req, wam_dmp_controller::ImpedanceControllerGains::Response &res)
    {
        // First check the size of the request 
        if (req.Kp_gains.size() != 6)
        {
            ROS_WARN("Kp_gains size is not 6, but %lu instead.", req.Kp_gains.size());
            res.accepted = false;
        }
        else if (req.Kv_gains.size() != 6)
        {
            ROS_WARN("Kv_gains size is not 6, but %lu instead.", req.Kv_gains.size());
            res.accepted = false;
        }
        else if (req.null_Kp_gains.size() != 6)
        {
            ROS_WARN("null_Kp_gains size is not 6, but %lu instead.", req.null_Kp_gains.size());
            res.accepted = false; 
        }
        else if (req.null_Kv_gains.size() != 6)
        {
            ROS_WARN("null_Kv_gains size is not 6, but %lu instead.", req.null_Kv_gains.size());
            res.accepted = false; 
        }
        else
        {
            for (int i = 0; i < 6; i++)
            {
                Kp_.coeffRef(i, i) = req.Kp_gains[i];
                Kv_.coeffRef(i, i) = req.Kv_gains[i];
                null_Kp_.coeffRef(i, i) = req.null_Kp_gains[i];
                null_Kv_.coeffRef(i, i) = req.null_Kv_gains[i];
            }
            res.accepted = true;
        }

        return res.accepted;   
    }

    bool OperationalSpaceImpedanceController::get_cmd_gains(wam_dmp_controller::ImpedanceControllerGains::Request &req, 
                                                            wam_dmp_controller::ImpedanceControllerGains::Response &res)
    {
        for (int i = 0; i < 6; i++)
        {
            res.Kp_gains.push_back(Kp_.coeff(i, i));
            res.Kv_gains.push_back(Kv_.coeff(i, i));
            res.null_Kp_gains.push_back(null_Kp_.coeff(i, i));
            res.null_Kv_gains.push_back(null_Kv_.coeff(i, i));
        }
        res.accepted = true;
        return res.accepted;
    }



    void OperationalSpaceImpedanceController::setCommandRT(Eigen::Vector3d trans_des, Eigen::Vector3d trans_dot_des, Eigen::Vector3d trans_dotdot_des, 
                      Eigen::Vector3d rot_des, Eigen::Vector3d rot_dot_des, Eigen::Vector3d rot_dotdot_des)
    {
        command_struct_.trans_xyz_command_ = trans_des;
        command_struct_.rot_xyz_command_ = rot_des;
        command_struct_.trans_xyzdot_command_ = trans_dot_des;
        command_struct_.rot_xyzdot_command_ = rot_dot_des;
        command_struct_.trans_xyzdotdot_command_ = trans_dotdot_des;
        command_struct_.rot_xyzdotdot_command_ = rot_dotdot_des;

        command_buffer_.initRT(command_struct_);
    }
    
    void OperationalSpaceImpedanceController::set_p_sensor_cp(double x, double y, double z)
    {
        p_sensor_cp_ = KDL::Vector(x, y, z);
    }

    void OperationalSpaceImpedanceController::get_gains_null(Eigen::Matrix<double, 6, 6>& null_Kp, 
                                                            Eigen::Matrix<double, 6, 6>& null_Kv)
    {
        null_Kp = null_Kp_;
        null_Kv = null_Kv_;
    }

    void OperationalSpaceImpedanceController::set_gains_null(Eigen::Matrix<double, 6, 6> null_Kp, 
                                                            Eigen::Matrix<double, 6, 6> null_Kv)
    {
        null_Kp_ = null_Kp;
        null_Kv_ = null_Kv;
    }
    
    void OperationalSpaceImpedanceController::set_p_wrist_ee(double x, double y, double z)
    {
        p_wrist_ee_ = KDL::Vector(x, y, z);
    }
  
    void OperationalSpaceImpedanceController::set_p_base_ws(double x, double y, double z)
    {
        p_base_ws_ = KDL::Vector(x, y, z);
    }

    void OperationalSpaceImpedanceController::set_ws_base_angles(double alpha, double beta, double gamma)
    {
        R_ws_base_ = KDL::Rotation::EulerZYX(alpha, beta, gamma);
    }
}
PLUGINLIB_EXPORT_CLASS(wam_dmp_controller::OperationalSpaceImpedanceController, controller_interface::ControllerBase)



