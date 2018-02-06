/*************************************************************************
	> File Name: dmp_ik_controller.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 13 Jun 2017 03:34:29 PM PDT
 ************************************************************************/

#ifndef _DMP_IK_CONTROLLER_H
#define _DMP_IK_CONTROLLER_H

// system includes 
#include <boost/shared_ptr.hpp>

// ros includes 
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

// local includes 
#include <wam_dmp_controller/joint_position_controller.h>
#include <wam_dmp_controller/dmp_controller.h>
#include <wam_dmp_controller/cartesian_twist_controller_ik_with_nullspace_optimization.h>
#include <wam_dmp_controller/circular_message_buffer.h>

#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

namespace wam_dmp_controller
{
    /*! \class controller that uses cartesian space set points and sends them to the pose twist controller. 
     */
    class DMPIKController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        typedef CartesianTwistControllerWithNullspaceOptimization CartController;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            /*! Constructor
             */
            DMPIKController();

            /*! Destructor
             */
            virtual ~DMPIKController() {};

            /*!
             * @param hw
             * @param node_handle
             * @return
             */
            bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &node_handle);

            /*!
             */
            void starting(const ros::Time& time);

            /*!
             */
            void update(const ros::Time& time, const ros::Duration& period);

            /*!
             */
        void stopping(const ros::Time& time);

            /*!
             * @return 
             */
            bool initXml(hardware_interface::EffortJointInterface *hw,
                         TiXmlElement* config);

            /*!
             * @return 
             */
            std::string getVariableNamesKeyWord() const;
            std::vector<int> getNumDofs() const;
        
        private:
            
            /*!
             * @param trajectory_point
             * @param movement_finished 
             * @param execution_duration
             * @param num_samples 
             * @return 
             */
            bool transformCommand(Eigen::VectorXd& trajectory_input,
                                  bool& movement_finished,
                                  const double execution_duration,
                                  const int num_samples);

            /*!
             * @return 
             */
            bool readParameters();

            /*!
             * @return 
             */
            bool initRTPublisher();

            /*!
             * @param handle_namespace
             * @param controller_handle_namespace
             * @return 
             */
            bool getArmRelatedVariables(const std::string& handle_namespace,
                                        std::string& controller_handle_namespace);

            /*!
             */
            bool initialized_;
            bool first_time_;

            /*! hardware_interface
             */
            hardware_interface::EffortJointInterface* hw_;

            /*!
             */
            std::string root_name_;

            /*!
             */
            ros::NodeHandle node_handle_;

            /*! IK controller 
             */
            boost::shared_ptr<CartController> cart_controller_;

            /*!
             */
            void visualize();

            /*!
             * @param input_vector
             * @param output_vector
             * @return True on success, otherwise False 
             * REAL-TIME REQUIREMENTS
             */
            bool adjustVariables(const Eigen::VectorXd& input_vector, Eigen::VectorXd& output_vector);

            /*!
             * REAL-TIME REQUIREMENTS
             * @return True on success, otherwise False 
             */
            bool setDesiredState();

            /*!
             * REAL-TIME REQUIREMENTS
             * @return True on success, otherwise False 
             */
            bool holdPositions();

            /*!
             * REAL-TIME REQUIREMENTS
             * @return True on success, otherwise False 
             */
            bool getDesiredPosition();

            /*!
             */
            int publishing_rate_;
            int publishing_counter_;
            int publisher_buffer_size_;

            boost::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker> > viz_marker_actual_arrow_publisher_;
            boost::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker> > viz_marker_desired_arrow_publisher_;
            boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> > pose_actual_publisher_;
            boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> > pose_desired_publisher_;

            int visualization_line_counter_;
            int visualization_line_rate_;
            int visualization_line_max_points_;

            boost::shared_ptr<CircularMessageBuffer<geometry_msgs::Point> > actual_line_points_;
            boost::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker> > viz_marker_actual_line_publisher_;
            boost::shared_ptr<CircularMessageBuffer<geometry_msgs::Point> > desired_line_points_;
            boost::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker> > viz_marker_desired_line_publisher_;
            int publishing_seq_counter_;

            /*!
             */
            bool keep_restposture_fixed_for_testing_;

            /*!
             */
            bool last_frame_set_;
            KDL::Frame last_frame_;
            KDL::Frame current_frame_;
            KDL::Twist current_twist_;

            /*!
             */
            int num_joints_;
            Eigen::VectorXd desired_positions_;
            Eigen::VectorXd desired_velocities_;
            Eigen::VectorXd desired_accelerations_;

            Eigen::VectorXd goal_;
            Eigen::VectorXd start_;
            Eigen::VectorXd local_vector_;

            //DMPController 
            boost::shared_ptr<DMPController> dmp_controller_;
            bool execution_error_;

            bool isIdle()
            {
                return dmp_controller_->isIdle();
            }
    };
}


#endif
