/*************************************************************************
	> File Name: operational_space_impedance_spline_controller.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Sun 25 Feb 2018 11:03:15 PM PST
 ************************************************************************/

#include <pluginlib/class_list_macros.h>
#include <wam_dmp_controller/operational_space_impedance_spline_controller.h>
#include <math.h>
#include <angles/angles.h>
#include <geometry_msgs/WrenchStamped.h>

#define DEFAULT_P2P_TRAJ_DURATION 5.0
#define P2P_COEFF_3 10.0
#define P2P_COEFF_4 -15.0
#define P2P_COEFF_5 6.0

namespace wam_dmp_controller
{
    OperationalSpaceImpedanceSplineController::OperationalSpaceImpedanceSplineController() {}
    ~OperationalSpaceImpedanceSplineController::OperationalSpaceImpedanceSplineController() {}

    bool OperationalSpaceImpedanceSplineController:: init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
    {
        OperationalSpaceImpedanceController::init(hw, n);

        set_cmd_traj_pos_service_ = n.advertiseService("set_traj_pos_cmd",
                                                       &OperationalSpaceImpedanceSplineController::set_cmd_traj_spline_srv, this);
        get_cmd_traj_pos_service_; = n.advertiseService("set_traj_pos_cmd",
                                                       &OperationalSpaceImpedanceSplineController::get_cmd_traj_spline_srv, this);

        // set trajectory duration
        p2p_traj_spline_duration_ = DEFAULT_P2P_TRAJ_DURATION;

        p2p_traj_const_ = Eigen::MatrixXf(4, 6);

        return true;
    }

    void OperationalSpaceImpedanceSplineController::starting(const ros::Time& time)
    {
        OperationalSpaceImpedanceController::starting(time);
        /*
            Important!!!!!!!!!!!!!!!!!!!
            whenever the controller is started the set point is set
            to the current configuration (For safety!!!!!!!!!)
            !!!!!!!!!!!!!!!!!!!!!!!!!
        */

        // set default trajectory 
        set_default_pos_traj(); 
    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
        Eigen::VectorXd x_des(6);
        Eigen::VectorXd xdot_des(6);
        Eigen::VectorXd xdotdot_des(6);
        eval_current_point_to_point_traj(period, x_des, xdot_des, xdotdot_des);
        OperationalSpaceImpedanceController::setCommandRT(x_des, xdot_des, xdotdot_des);
        OperationalSpaceImpedanceController::update(time, period);
    }

    void HybridImpedanceController::set_default_pos_traj()
    {
        // get current robot joints configuration q
        KDL::JntArray q;
        q.resize(kdl_chain_.getNrOfJoints());
        for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
            q(i) = joint_handles_[i].getPosition();
    
        // forward kinematics
        KDL::Frame ee_fk_frame;
        ee_fk_solver_->JntToCart(q, ee_fk_frame);
    
        // evaluate current cartesian configuration
        KDL::Rotation R_ws_ee;
        KDL::Vector p_ws_ee;
        double alpha, beta, gamma;
        R_ws_ee = R_ws_base_ * ee_fk_frame.M;
        R_ws_ee.GetEulerZYX(alpha, beta, gamma);
        p_ws_ee = R_ws_base_ * (ee_fk_frame.p - p_base_ws_);

        // set position and attitude tajectory constants
        for(int i=0; i<6; i++)
        {
	        p2p_traj_const_(1, i) = 0;
	        p2p_traj_const_(2, i) = 0;
	        p2p_traj_const_(3, i) = 0;
        }

        for(int i=0; i<3; i++)
            p2p_traj_const_(0, i) = p_ws_ee.data[i];
        p2p_traj_const_(0, 3) = gamma;
        p2p_traj_const_(0, 4) = beta;
        p2p_traj_const_(0, 5) = alpha;
        prev_trans_setpoint_ << p_ws_ee.x(), p_ws_ee.y(), p_ws_ee.z();
        prev_rot_setpoint_ << alpha, beta, gamma;

        // reset the time
        time_ = p2p_traj_duration_;

        p2p_traj_mutex_.unlock();
    }
}

