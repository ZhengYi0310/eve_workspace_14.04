/*************************************************************************
	> File Name: operational_space_impedance_spline_controller.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Sun 25 Feb 2018 10:46:33 PM PST
 ************************************************************************/

#ifndef __WAM_DMP_OPERATIONAL_SPACE_IMPEDANCE_SPLINE_CONTROLLER_H
#define __WAM_DMP_OPERATIONAL_SPACE_IMPEDANCE_SPLINE_CONTROLLER_H

#include "wam_dmp_controller/operational_space_impedance_controller.h"
#include <wam_dmp_controller/PoseRPYCommand.h>
//#include <wam_dmp_controller/GainsCommand.h>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

namespace wam_dmp_controller
{
    class OperationalSpaceImpedanceSplineController : public wam_dmp_controller::OperationalSpaceImpedanceController
    {
        public:
            OperationalSpaceImpedanceSplineController();
            ~OperationalSpaceImpedanceSplineController();

            bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  			void starting(const ros::Time& time);
  			void update(const ros::Time& time, const ros::Duration& period);


        private:
            void set_default_pos_traj();
            bool set_cmd_traj_spline_srv(wam_dmp_controller::PoseRPYCommand::Request &req, 
                                         wam_dmp_controller::PoseRPYCommand::Response &res);
            bool get_cmd_traj_spline_srv(wam_dmp_controller::PoseRPYCommand::Request &req,
                                     wam_dmp_controller::PoseRPYCommand::Response &res);

            //void get_parameters(ros::NodeHandle &n);
            void eval_current_point_to_point_traj(const ros::Duration& period,
                                                  Eigen::Vector3d& trans_des,
                                                  Eigen::Vector3d& trans_dot_des,
                                                  Eigen::Vector3d& trans_dotdot_des, 
                                                  Eigen::Vector3d& rot_des, 
                                                  Eigen::Vector3d& rot_dot_des, 
                                                  Eigen::Vector3d& rot_dotdot_des);

            void eval_point_to_point_traj_constants(Eigen::Vector3d& desired_trans,
                                                    Eigen::Vector3d& desired_rot,
                                                    double duration);

            ros::ServiceServer set_cmd_traj_pos_service_;     
            ros::ServiceServer get_cmd_traj_pos_service_;

            Eigen::Vector3d trans_des_;
            Eigen::Vector3d trans_dot_des_;
            Eigen::Vector3d trans_dotdot_des_;
            Eigen::Vector3d rot_des_;
            Eigen::Vector3d rot_dot_des_;
            Eigen::Vector3d rot_dotdot_des_;

            double p2p_traj_spline_duration_;
            Eigen::MatrixXf p2p_traj_const_;
            Eigen::Vector3d prev_trans_setpoint_;
            Eigen::Vector3d prev_rot_setpoint_;
            double time_;
                
            //boost::shared_ptr<OperationalSpaceImpedanceController> ops_imp_controller_;
            boost::mutex p2p_traj_mutex_;
    };
}
#endif
