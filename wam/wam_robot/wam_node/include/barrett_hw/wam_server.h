/*************************************************************************
	> File Name: wam_server.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 01 Jun 2017 11:52:35 AM PDT
 ************************************************************************/

#ifndef _WAM_SERVER_H
#define _WAM_SERVER_H
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

//#include <barrett_model/semi_absolute_joint_interface.h>

#include <terse_roscpp/param.h>

#include <urdf/model.h>

#include <stdexcept>

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
            BarrettHW(ros::NodeHandle nh);
            bool configure();
            bool start();
            bool read(const ros::Time time, const ros::Duration period);
            void write(const ros::Time time, const ros::Duration period);
            void stop()
            void cleanup() {}

            // Wait for all devices to become available
            bool wait_for_mode(barrett::SafetyModule::SafetyMode mode, ros::Duration timeout = ros::Duration(60, 0), ros::Duration poll_duration = ros::Duration(0.1))

            void set_mode(barrett::SafetyModule::SafetyMode mode);

            // State structure for a Wam arm 
            // This provides storage for the joint handles
            template<size_t DOF>
            struct WamDevice
            {
                //  Systems Wam and its Low-level Wam interface 
                //boost::shared_ptr<barrett::systems::Wam<DOF> > Wam;
                //boost::shared_ptr<barrett::LowLevelWam<DOF> >interface;
                barrett::systems::Wam<DOF>* Wam;
                barrett::LowLevelWam<DOF>& interface;
                
                // Configuration 
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

                //***********8
                boost::shared_ptr<BarrettHW::HandDevice> hand_device;
            };

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
                Eigen::vector4i calibrated_joints;
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

            // ros_control interface 
            hardware_interface::JointStateInterface state_interface_;
            hardware_interface::EffortJointInterface effort_interface_;
            hardware_interface::VelocityJointInterface velocity_interface_;
            barrett_model::SemiAbsoluteJointInterface semi_absolute_interface_;

            // Vectors of various barrett structures 
            ManagerMap barrett_managers_;
            Wam4Map wam4s_;
            Wam7Map wam7s_;
            HandMap hands_;

            jp_type jp_home;
            barrett::systems::Ramp ramp;

        protected:

            template<size_t DOF>
            //Eigen::Matrix<double, DOF, 1> compute_resolver_ranges(boost::shared_ptr<barrett::LowLevelWam<DOF> > wam);
            Eigen::Matrix<double, DOF, 1> compute_resolver_ranges(barrett::LowLevelWam<DOF>& wam);

            template<size_t DOF>
            boost::shared_ptr<BarrettHW::WamDevice<DOF> > configure_wam(ros::NodeHandle product_nh, boost::shared_ptr<barrett::ProductManager> barrett_manager, const libconfig::Setting &wam_config);

            template<size_t DOF>
            bool read_wam(const ros::Time time, const ros::Duration period, boost::shared_ptr<BarrettHW::WamDevice<DOF> > device);

            template<size_t DOF>
            void write_wam(const ros::Time time, const ros::Duration period, boost shared_ptr<BarrettHW::WamDevice<DOF> > device);
    };
}
#endif

