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
    OperationalSpaceImpedanceSplineController::~OperationalSpaceImpedanceSplineController() {}

    bool OperationalSpaceImpedanceSplineController:: init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
    {
        OperationalSpaceImpedanceController::init(hw, n);

        set_cmd_traj_pos_service_ = n.advertiseService("set_traj_pos_cmd",
                                                       &OperationalSpaceImpedanceSplineController::set_cmd_traj_spline_srv, this);
        get_cmd_traj_pos_service_ = n.advertiseService("set_traj_pos_cmd",
                                                       &OperationalSpaceImpedanceSplineController::get_cmd_traj_spline_srv, this);

        // set trajectory duration
        p2p_traj_spline_duration_ = DEFAULT_P2P_TRAJ_DURATION;

        p2p_traj_const_ = Eigen::MatrixXf(4, 6);

        trans_des_          = Eigen::Vector3d::Zero();
        trans_dot_des_       = Eigen::Vector3d::Zero();
        trans_dotdot_des_    = Eigen::Vector3d::Zero();
        rot_des_            = Eigen::Vector3d::Zero();
        rot_dot_des_        = Eigen::Vector3d::Zero();
        rot_dotdot_des_     = Eigen::Vector3d::Zero();

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

    void OperationalSpaceImpedanceSplineController::update(const ros::Time& time, const ros::Duration& period)
    {
        Eigen::Vector3d x_des(6);
        Eigen::VectorXd xdot_des(6);
        Eigen::VectorXd xdotdot_des(6);
        eval_current_point_to_point_traj(period, trans_des_, trans_dot_des_, trans_dotdot_des_, 
                                         rot_des_, rot_dot_des_, rot_dotdot_des_);
        OperationalSpaceImpedanceController::setCommandRT(trans_des_, trans_dot_des_, trans_dotdot_des_, 
                                                          rot_des_, rot_dot_des_, rot_dotdot_des_);
        OperationalSpaceImpedanceController::update(time, period);
    }

    void OperationalSpaceImpedanceSplineController::set_default_pos_traj()
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
        time_ = p2p_traj_spline_duration_;

        p2p_traj_mutex_.unlock();
    }

    bool OperationalSpaceImpedanceSplineController::set_cmd_traj_spline_srv(wam_dmp_controller::PoseRPYCommand::Request &req, 
                                                                            wam_dmp_controller::PoseRPYCommand::Response &res)
    {
        if (time_ < p2p_traj_spline_duration_)
        {
            res.command.elapsed_time = time_;
            res.command.accepted = false;
            res.command.p2p_traj_duration = p2p_traj_spline_duration_;
            
            return true;
        }
        res.command.accepted = true;

        // set requested position and attitude
        command_struct_.trans_xyz_command_[0] = req.command.position.x;
        command_struct_.trans_xyz_command_[1] = req.command.position.y;
        command_struct_.trans_xyz_command_[2] = req.command.position.z;
        command_struct_.rot_xyz_command_[0] = req.command.orientation.roll;
        command_struct_.rot_xyz_command_[1] = req.command.orientation.yaw;
        command_struct_.rot_xyz_command_[2] = req.command.orientation.pitch;

        p2p_traj_mutex_.lock();

        p2p_traj_spline_duration_ = req.command.p2p_traj_duration;
        //command_buffer_.writeFromNonRT(command_struct_); Don't do this when set a spline!!!!!! 
        eval_point_to_point_traj_constants(command_struct_.trans_xyz_command_, command_struct_.rot_xyz_command_, p2p_traj_spline_duration_);
        time_ = 0;

        p2p_traj_mutex_.unlock();

        return true; 
    }
    
    bool OperationalSpaceImpedanceSplineController::get_cmd_traj_spline_srv(wam_dmp_controller::PoseRPYCommand::Request &req,
                                 wam_dmp_controller::PoseRPYCommand::Response &res)
    {
        // get translation
        res.command.position.x = prev_trans_setpoint_[0];
        res.command.position.y = prev_trans_setpoint_[1];
        res.command.position.z = prev_trans_setpoint_[2];
        res.command.p2p_traj_duration = p2p_traj_spline_duration_;

        // get rotation
        res.command.orientation.roll = prev_rot_setpoint_[0];
        res.command.orientation.yaw = prev_rot_setpoint_[1];
        res.command.orientation.pitch = prev_rot_setpoint_[2];

        // get elapsed time
        res.command.elapsed_time = time_;

        return true;        
    }

    void OperationalSpaceImpedanceSplineController::eval_current_point_to_point_traj(const ros::Duration& period,
                                          Eigen::Vector3d& trans_des,
                                          Eigen::Vector3d& trans_dot_des,
                                          Eigen::Vector3d& trans_dotdot_des,
                                          Eigen::Vector3d& rot_des, 
                                          Eigen::Vector3d& rot_dot_des, 
                                          Eigen::Vector3d& rot_dotdot_des)
    {
        p2p_traj_mutex_.lock();
        time_ += period.toSec();

        if (time_ > p2p_traj_spline_duration_)
        {    
            time_ = p2p_traj_spline_duration_;
            //run_spline_ = false;
        }

        for (int i=0; i<3; i++)
        {
	        trans_des[i] = p2p_traj_const_(0, i) + p2p_traj_const_(1, i) * pow(time_, 3) + \
                       p2p_traj_const_(2, i) * pow(time_, 4) + p2p_traj_const_(3, i) * pow(time_, 5);

	        trans_dot_des[i] = 3 * p2p_traj_const_(1, i) * pow(time_, 2) + \
                          4 * p2p_traj_const_(2, i) * pow(time_, 3) + 5 * p2p_traj_const_(3, i) * pow(time_, 4);

	        trans_dotdot_des[i] = 3 * 2 *  p2p_traj_const_(1, i) * time_ + \
                             4 * 3 * p2p_traj_const_(2, i) * pow(time_, 2) + 5 * 4 * p2p_traj_const_(3, i) * pow(time_, 3);
        }
        for (int i = 3; i < 6; i++)
        {
            rot_des[i] = p2p_traj_const_(0, i) + p2p_traj_const_(1, i) * pow(time_, 3) + \
                       p2p_traj_const_(2, i) * pow(time_, 4) + p2p_traj_const_(3, i) * pow(time_, 5);

	        rot_dot_des[i] = 3 * p2p_traj_const_(1, i) * pow(time_, 2) + \
                          4 * p2p_traj_const_(2, i) * pow(time_, 3) + 5 * p2p_traj_const_(3, i) * pow(time_, 4);

	        rot_dotdot_des[i] = 3 * 2 *  p2p_traj_const_(1, i) * time_ + \
                             4 * 3 * p2p_traj_const_(2, i) * pow(time_, 2) + 5 * 4 * p2p_traj_const_(3, i) * pow(time_, 3);
        }
        p2p_traj_mutex_.unlock();
    }

    void OperationalSpaceImpedanceSplineController::eval_point_to_point_traj_constants(Eigen::Vector3d& desired_trans,
                                                                                       Eigen::Vector3d& desired_rot,
                                                                                       double duration)
    {
        // evaluate common part of constants
        double constant_0, constant_1, constant_2;
        constant_0 = P2P_COEFF_3 / pow(duration, 3);
        constant_1 = P2P_COEFF_4 / pow(duration, 4);
        constant_2 = P2P_COEFF_5 / pow(duration, 5);

        // evaluate constants for x and y trajectories
        for (int i=0; i<3; i++)
        {
	        double error = desired_trans(i) - prev_trans_setpoint_(i);
	        p2p_traj_const_(0, i) = prev_trans_setpoint_(i);
	        p2p_traj_const_(1, i) = error * constant_0;
	        p2p_traj_const_(2, i) = error * constant_1;
	        p2p_traj_const_(3, i) = error * constant_2;
        }
        prev_trans_setpoint_ = desired_trans;

        // evaluate constants alpha, beta and gamma trajectories
        double alpha_cmd, beta_cmd, gamma_cmd;
        KDL::Rotation::EulerZYX(desired_rot(2),
			                    desired_rot(1),
			                    desired_rot(0)).GetEulerZYX(alpha_cmd, beta_cmd, gamma_cmd);
        Eigen::Vector3d des_attitude_fixed;
        des_attitude_fixed << gamma_cmd, beta_cmd, alpha_cmd;
        for (int i=0; i<3; i++)
        {
            double error = angles::normalize_angle(des_attitude_fixed(i) - prev_rot_setpoint_(i));
	        p2p_traj_const_(0, i + 3) = prev_rot_setpoint_(i);
	        p2p_traj_const_(1, i + 3) = error * constant_0;
	        p2p_traj_const_(2, i + 3) = error * constant_1;
	        p2p_traj_const_(3, i + 3) = error * constant_2;
        }
        prev_rot_setpoint_ = des_attitude_fixed;
    }
}
PLUGINLIB_EXPORT_CLASS(wam_dmp_controller::OperationalSpaceImpedanceSplineController, controller_interface::ControllerBase)
