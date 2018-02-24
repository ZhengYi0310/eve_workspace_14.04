#include <pluginlib/class_list_macros.h>
#include "wam_dmp_controller/euler_kinematic_rpy.h"
#include <angles/angles.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include "wam_dmp_controller/operational_space_controller.h"
#include "wam_dmp_controller/effective_mass_matrix.hpp"


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
	OperationalSpaceController::OperationalSpaceController() {};
  	OperationalSpaceController::~OperationalSpaceController() {};

  	bool OperationalSpaceController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  	{
  		KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(hw, n);

        //extended_chain_(n);
        
        ros::NodeHandle Kp_handle(n, "Kp_Gains");
        ros::NodeHandle Kv_handle(n, "Kv_Gains");
        ros::NodeHandle Ki_handle(n, "Ki_Gains");
        ros::NodeHandle null_Kp_handle(n, "null_Kp_gains");
        ros::NodeHandle null_Kv_handle(n, "null_Kv_gains");
        
        for (size_t i = 0; i < joint_handles_.size(); i++)
        {
            std::cout << joint_handles_[i].getName();
            if ( !Kp_handle.getParam(joint_handles_[i].getName(), Kp_(i) ) ) 
            {
                ROS_WARN("Kp gain not set for %s in yaml file, Using 300.", joint_handles_[i].getName().c_str());
                Kp_.coeffRef(i, i) = 300;
            }
            else 
            {
                ROS_WARN("Kp gain set for %s in yaml file, Using %f", joint_handles_[i].getName().c_str(), Kp_(i)); 
            }

            if ( !Kv_handle.getParam(joint_handles_[i].getName().c_str(), Kv_(i) ) ) 
            {
                ROS_WARN("Kv gain not set for %s in yaml file, Using 0.7.", joint_handles_[i].getName().c_str());
                Kv_.coeffRef(i, i) = 0.7;
            }
            else 
            {
                 ROS_WARN("Kv gain set for %s in yaml file, Using %f", joint_handles_[i].getName().c_str(), Kv_(i)); 
            }     
            // scale Kp_ gains for later 
            Kp_.coeffRef(i, i) /= Kv_.coeffRef(i, i);
            

            if ( !Ki_handle.getParam(joint_handles_[i].getName().c_str(), Ki_(i) ) ) 
            {
                ROS_WARN("Ki gain not set for %s in yaml file, Using 0.5.", joint_handles_[i].getName().c_str());
                Kv_.coeffRef(i, i) = 0.5;
            }
            else 
            {
                 ROS_WARN("Ki gain set for %s in yaml file, Using %f", joint_handles_[i].getName().c_str(), Kv_(i)); 
            } 

            if ( !null_Kp_handle.getParam(joint_handles_[i].getName().c_str(), null_Kp_(i) ) ) 
            {
                ROS_WARN("null_Kp gain not set for %s in yaml file, Using 0.0.", joint_handles_[i].getName().c_str());
                null_Kp_.coeffRef(i, i) = 0.0;
            }
            else 
            {
                 ROS_WARN("null_Kp gain set for %s in yaml file, Using %f", joint_handles_[i].getName().c_str(), null_Kp_(i)); 
            } 

            if ( !null_Kv_handle.getParam(joint_handles_[i].getName().c_str(), null_Kv_(i) ) ) 
            {
                ROS_WARN("null_Kv gain not set for %s in yaml file, Using 0.0.", joint_handles_[i].getName().c_str());
                null_Kv_.coeffRef(i, i) = 0.0;
            }
            else 
            {
                 ROS_WARN("null_Kv gain set for %s in yaml file, Using %f", joint_handles_[i].getName().c_str(), null_Kv_(i)); 
            } 

            //TODO also need to load the rest joint postures from the parameter server.....
            
            
        }

        tau_.resize(6);
        M.resize(kdl_chain_.getNrOfJoints());
        C.resize(kdl_chain_.getNrOfJoints());
        G.resize(kdl_chain_.getNrOfJoints());
        command_struct_.trans_xyz_command_ = Eigen::Vector3d::Zero();
        command_struct_.rot_xyz_command_ = Eigen::Vector3d::Zero();
        command_struct_.trans_xyzdot_command_ = Eigen::Vector3d::Zero();
        command_struct_.rot_xyzdot_command_ = Eigen::Vector3d::Zero();
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
        trans_xyz_ws_ = Eigen::Vector3d::Zero();
        trans_xyzdot_ws_ = Eigen::Vector3d::Zero();
        rot_xyz_ws_ = Eigen::Vector3d::Zero();
        rot_xyzdot_ws_ = Eigen::Vector3d::Zero();
        trans_xyz_des_ws_ = Eigen::Vector3d::Zero();
        trans_xyzdot_des_ws_ = Eigen::Vector3d::Zero();
        rot_xyz_des_ws_ = Eigen::Vector3d::Zero();
        rot_xyzdot_des_ws_ = Eigen::Vector3d::Zero();
        ws_x_ = Eigen::VectorXd(6);
        ws_xdot_ = Eigen::VectorXd(6);
        tau_ = Eigen::VectorXd(kdl_chain_.getNrOfJoints());
        q_rest_ = Eigen::VectorXd(kdl_chain_.getNrOfJoints());
        command_filter_ = Eigen::MatrixXd::Zero(kdl_chain_.getNrOfJoints(), 6);
        F_unit_ = Eigen::VectorXd(6);
        J_dyn_inv = Eigen::MatrixXd::Zero(6, 6);

    	// instantiate analytical to geometric transformation matrices
    	ws_E_ = Eigen::MatrixXd::Zero(6,6);
    	ws_E_.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3>::Identity();
    	ws_E_dot_ = Eigen::MatrixXd::Zero(6,6);

    	null_Kp_ = Eigen::MatrixXd::Zero(6, 6);
    	null_Kv_ = Eigen::MatrixXd::Zero(6, 6);
    	lamb_ = Eigen::MatrixXd::Zero(6, 6);
      	// Start command subscriber 
        sub_command_ = n.subscribe("Cartesian_space_command", 10, &OperationalSpaceController::setCommandCB, this);

      	return true;
  	}

  	void OperationalSpaceController::starting(const ros::Time& time)
  	{

  		command_buffer_.initRT(command_struct_);
  		// get current robot configuration (q and q dot)
    	for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
      	{
			joint_msr_states_.q(i) = joint_handles_[i].getPosition();
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
      	}

   		return;
  	}

    void OperationalSpaceController::update(const ros::Time& time, const ros::Duration& period)
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
        trans_xyz_des_ws_ = command_struct_.trans_xyz_command_;
        trans_xyzdot_des_ws_ = command_struct_.trans_xyzdot_command_;
        rot_xyz_des_ws_ = command_struct_.rot_xyz_command_;
        rot_xyzdot_des_ws_ = command_struct_.rot_xyzdot_command_;
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
    	//        zeros(3), inv(T(PHI))]
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
      		Lambda_inv = ws_JA_ee * M.data.inverse() * ws_JA_ee.transpose();
    	else
      		// use kdl matrix for real scenario
      		Lambda_inv = ws_JA_ee * M.data.inverse() * ws_JA_ee.transpose();

    	ComputeMassMatrix(Lambda_inv, Lambda);

        // Always consider null-space control, otherwise might cause unstable behavior for redundant Manipulator 
        // in joint space 
        J_dyn_inv = M.data.inverse() * ws_JA_ee.transpose() * Lambda;

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
    	//
      	command_filter_ = ws_JA_ee.transpose() * Lambda;        
    	if(use_simulation_)
      		tau_ = C.data * joint_msr_states_.qdot.data + G.data - command_filter_ * ws_JA_ee_dot * joint_msr_states_.qdot.data;
    	else
      		tau_ = C.data * joint_msr_states_.qdot.data + G.data - command_filter_ * ws_JA_ee_dot * joint_msr_states_.qdot.data;
    	//////////////////////////////////////////////////////////////////////////////////
    	//
    	// 
    	// (see A Unified Approach for Motion and Force Control
    	// of Robot Manipulators: The Operational Space Formulation, Oussama Khatib
    	// for details on the definition of a dynamically consistent generalized inverse)
    	//
    	//////////////////////////////////////////////////////////////////////////////////
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

    void OperationalSpaceController::setCommand(geometry_msgs::Vector3 position, wam_dmp_controller::RPY orientation)
    {
        command_struct_.trans_xyz_command_[0] = position.x;
        command_struct_.trans_xyz_command_[1] = position.y;
        command_struct_.trans_xyz_command_[2] = position.z;
        command_struct_.rot_xyz_command_[0] = orientation.roll;
        command_struct_.rot_xyz_command_[1] = orientation.pitch;
        command_struct_.rot_xyz_command_[2] = orientation.yaw;

        command_struct_.trans_xyzdot_command_ = Eigen::Vector3d::Zero();
        command_struct_.rot_xyzdot_command_ = Eigen::Vector3d::Zero();

        command_buffer_.writeFromNonRT(command_struct_);        
    }
  	// Start command subscriber 
    void OperationalSpaceController::setCommandCB(const wam_dmp_controller::PoseRPYConstPtr& msg)
    {
        setCommand(msg->position, msg->orientation);
    }

    void OperationalSpaceController::set_p_wrist_ee(double x, double y, double z)
    {
        p_wrist_ee_ = KDL::Vector(x, y, z);
    }
  
    void OperationalSpaceController::set_p_base_ws(double x, double y, double z)
    {
        p_base_ws_ = KDL::Vector(x, y, z);
    }

    void OperationalSpaceController::set_ws_base_angles(double alpha, double beta, double gamma)
    {
        R_ws_base_ = KDL::Rotation::EulerZYX(alpha, beta, gamma);
    }
}
PLUGINLIB_EXPORT_CLASS(wam_dmp_controller::OperationalSpaceController, controller_interface::ControllerBase)



