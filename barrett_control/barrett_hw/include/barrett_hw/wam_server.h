/*************************************************************************
	> File Name: wam_server.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 01 Jun 2017 11:52:35 AM PDT
 ************************************************************************/

#ifndef _WAM_SERVER_H
#define _WAM_SERVER_H
#define EIGEN2_SUPPORT
#define EIGEN2_SUPPORT_STAGE40_FULL_EIGEN3_STRICTNESS
#define BARRETT_XENOMAI

#include <unistd.h>
#include <ros/ros.h>
#include <native/task.h>
#include <native/timer.h>
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <controller_manager/controller_manager.h>
#include <signal.h>
#include <realtime_tools/realtime_publisher.h>
#include <control_toolbox/filters.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Duration.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/kinfam_io.hpp>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <libconfig.h++>

#include <barrett/exception.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/math.h> 
#include <barrett/systems.h>
#include <barrett/bus/can_socket.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems/wam.h>

#include <barrett_model/semi_absolute_joint_interface.h>
#include <barrett_model/biotac_finger_interface.h>
#include <barrett_model/arm_pose_interface.h>
#include <barrett_model/robot_state_interface.h>

#include <biotac_sensors/biotac_hand_class.h>
#include <biotac_sensors/BioTacHand.h>

#include <terse_roscpp/param.h>

#include <urdf/model.h>

#include <stdexcept>

//#include <usc_utilities/assert.h>

bool g_quit = false;

void quitRequested(int sig)
{
    g_quit = true;
}

namespace barrett_hw
{
    class BarrettHW : public hardware_interface::RobotHW
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            BarrettHW(ros::NodeHandle nh);
            bool configure();
            bool start();
            bool read(const ros::Time time, const ros::Duration period);
            void write(const ros::Time time, const ros::Duration period);
            void stop();
            void cleanup() {}
            bool assignKDLTree(KDL::Tree, size_t num_of_segments);
            void printLink(const KDL::SegmentMap::const_iterator& link, const std::string& prefix);
            

            // Wait for all devices to become available
            bool wait_for_mode(barrett::SafetyModule::SafetyMode mode, ros::Duration timeout = ros::Duration(60, 0), ros::Duration poll_duration = ros::Duration(0.1));

            void set_mode(barrett::SafetyModule::SafetyMode mode);

            // State structure for the hand 
            struct HandDevice
            {
                //boost::shared_ptr<barrett::Hand> interface;
                barrett::Hand* interface;

                // Configuration 
                std::vector<std::string> joint_names;
                Eigen::Vector4d resolver_ranges;

                // State 
                Eigen::Vector4d joint_positions, joint_velocities, joint_effort_cmds, joint_offsets, resolver_angles, calibration_burn_offsets;
                Eigen::Vector4i calibrated_joints;
            };

            
            enum {JT_INPUT = 0, GRAVITY_INPUT, SC_INPUT};
            // State structure for a Wam arm 
            // This provides storage for the joint handles
            template<size_t DOF>
            struct WamDevice
            {
                BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
                //  Systems Wam and its Low-level Wam interface 
                //boost::shared_ptr<barrett::systems::Wam<DOF> > Wam;
                boost::shared_ptr<barrett::systems::LowLevelWamWrapper<DOF> > Wam;
                boost::shared_ptr<barrett::systems::PIDController<jp_type, jt_type> > jpController;
                boost::shared_ptr<barrett::systems::FirstOrderFilter<jv_type> > jvFilter;
                boost::shared_ptr<barrett::systems::KinematicsBase<DOF> > kinematicsBase;
                boost::shared_ptr<barrett::systems::GravityCompensator<DOF> > gravityCompensator;
                boost::shared_ptr<barrett::systems::Summer<jt_type, 3> > jtSum;
                boost::shared_ptr<barrett::systems::ExposedOutput<jt_type> > ExposedOutput;

                boost::shared_ptr<barrett::systems::PIDController<jv_type, jt_type> > jvController1; //For now, define this just in order to get joint velocities 
                //boost::shared_ptr<barrett::systems::Wam<DOF> > Wam;
                //barrett::LowLevelWam<DOF>& interface;
                
                // Configuration
                std::string tip_seg_name, root_seg_name, tip_joint_name;
                std::vector<std::string> joint_names;
                Eigen::Matrix<double, DOF, 1> resolver_ranges, effort_limits, velocity_limits;

                // State 
                Eigen::Matrix<double, DOF, 1> joint_positions, joint_velocities, joint_effort_cmds, joint_offsets, resolver_angles, calibration_burn_offsets;

                Eigen::Matrix<int, DOF, 1> calibrated_joints;
                
                void set_zero()
                {
                    joint_positions.setZero();
                    joint_velocities.setZero();
                    joint_effort_cmds.setZero();
                    joint_offsets.setZero();
                    resolver_ranges.setZero();
                    calibration_burn_offsets.setZero();
                }

                //***********
                boost::shared_ptr<HandDevice> hand_device;
                
                // stuff for pose and twist
                boost::shared_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
                boost::shared_ptr<KDL::ChainFkSolverVel_recursive> jnt_to_twist_solver_;
                boost::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
                KDL::Jacobian kdl_chain_jacobian_;
                KDL::Chain kdl_chain_;                
                KDL::JntArray kdl_current_joint_positions_;
                KDL::JntArrayVel kdl_current_joint_velocities_;
                KDL::Frame kdl_pose_measured_;
                KDL::Twist kdl_twist_measured_;
                KDL::FrameVel frame_vel_measured_;
            };
            

            // typedefs to make the world less verbose 
            typedef WamDevice<4> WamDevice4;
            typedef WamDevice<7> WamDevice7;
            typedef std::map<std::string, boost::shared_ptr<barrett::ProductManager> > ManagerMap;
            typedef std::map<std::string, boost::shared_ptr<WamDevice4> > Wam4Map;
            typedef std::map<std::string, boost::shared_ptr<WamDevice7> > Wam7Map;
            typedef std::map<std::string, boost::shared_ptr<HandDevice> > HandMap;

        private:

            // State 
            ros::NodeHandle nh_;
            bool configured_;
            bool calibrated_;

            // Configuration
            urdf::Model urdf_model_;        
            std::string urdf_str_;
            std::string urdf_path_;
            KDL::Tree kdl_tree_;
            // ros_control interface 
            hardware_interface::JointStateInterface state_interface_;
            hardware_interface::EffortJointInterface effort_interface_;
            hardware_interface::VelocityJointInterface velocity_interface_;
            barrett_model::SemiAbsoluteJointInterface semi_absolute_interface_;
            barrett_model::BiotacHandStateInterface biotac_hand_interface_;
            barrett_model::ArmPoseStatesInterface arm_pose_state_interface_;
            barrett_model::RobotStateInterface robot_state_interface_;

            // Vectors of various barrett structures 
            ManagerMap barrett_managers_;
            Wam4Map wam4s_;
            Wam7Map wam7s_;
            HandMap hands_;

            //barrett::jp_type jp_home;
            barrett::systems::Ramp ramp;

            std::string config_path_;
            bool tactile_sensors_exist_;
            size_t joint_states_to_biotac_counter_;
            bool verbose_; // To decide if print out the real time publishing status

            boost::shared_ptr<realtime_tools::RealtimePublisher<biotac_sensors::BioTacHand> > debugging_biotac_pub_;
        protected:

            template<size_t DOF>
            //Eigen::Matrix<double, DOF, 1> compute_resolver_ranges(boost::shared_ptr<barrett::LowLevelWam<DOF> > wam);
            Eigen::Matrix<double, DOF, 1> compute_resolver_ranges(barrett::LowLevelWam<DOF>& wam);

            template<size_t DOF>
            boost::shared_ptr<BarrettHW::WamDevice<DOF> > configure_wam(ros::NodeHandle product_nh, boost::shared_ptr<barrett::ProductManager> barrett_manager, const libconfig::Setting &wam_config);

            template<size_t DOF>
            bool read_wam(const ros::Time time, const ros::Duration period, boost::shared_ptr<BarrettHW::WamDevice<DOF> > device);

            template<size_t DOF>
            void write_wam(const ros::Time time, const ros::Duration period, boost::shared_ptr<BarrettHW::WamDevice<DOF> > device);
    };
}
#endif

