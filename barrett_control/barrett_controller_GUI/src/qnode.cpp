/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/barrett_controller_GUI/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace barrett_controller_GUI {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
        init_argc(argc),
        init_argv(argv)
        {}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
        wait();
}

bool QNode::init() {
        ros::init(init_argc,init_argv,"barrett_controller_switcher");
        if ( ! ros::master::check() ) {
                return false;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;
        // Add your ros communications here.
        chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
        start();
        sub_joints_state_ = n.subscribe("/joint_states", 1000,\
                                        &barrett_controller_GUI::QNode::joints_state_callback, this);

        sub_joints_error_ = n.subscribe("/" + robot_namespace_ + "/joint_space_spline_controller/jt_err", 1000,\
                                        &barrett_controller_GUI::QNode::joints_error_callback, this);


        sub_cart_error_ = n.subscribe("/" + robot_namespace_ + "/operational_space_impedance_spline_controller/cart_err", 1000,\
                                    &barrett_controller_GUI::QNode::cart_error_callback, this);
        sub_cart_pos_ = n.subscribe("/" + robot_namespace_ + "/operational_space_impedance_spline_controller/curr_cart_pos", 1000,\
                                    &barrett_controller_GUI::QNode::cart_pos_callback, this);
    /*
    sub_cartesian_error_ = n.subscribe("/" + robot_namespace_ + "/hybrid_impedance_controller/error", 1000,\
                       &barrett_controller_switcher::QNode::cartesian_error_callback, this);
    */
    ros::Duration(3).sleep();
    is_jointpos_controller_active_ = false;
    is_cartpos_controller_active_ = false;
        return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
        std::map<std::string,std::string> remappings;
        remappings["__master"] = master_url;
        remappings["__hostname"] = host_url;
        ros::init(remappings,"barrett_controller_switcher");
        if ( ! ros::master::check() ) {
                return false;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;
        // Add your ros communications here.
    //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    //set_robot_namespace("barrett_hw");
        start();
    sub_joints_state_ = n.subscribe("/joint_states", 1000,\
                        &barrett_controller_GUI::QNode::joints_state_callback, this);

    sub_joints_error_ = n.subscribe("/" + robot_namespace_ + "/joint_space_spline_controller/jt_err", 1000,\
                    &barrett_controller_GUI::QNode::joints_error_callback, this);

    sub_cart_pos_ = n.subscribe("/" + robot_namespace_ + "/operational_space_impedance_spline_controller/curr_cart_pos", 1000,\
                                  &barrett_controller_GUI::QNode::cart_pos_callback, this);
    sub_cart_error_ = n.subscribe("/" + robot_namespace_ + "/operational_space_impedance_spline_controller/cart_err", 1000,\
                                  &barrett_controller_GUI::QNode::cart_error_callback, this);
    /*
    sub_cartesian_error_ = n.subscribe("/" + robot_namespace_ + "/hybrid_impedance_controller/error", 1000,\
                       &barrett_controller_switcher::QNode::cartesian_error_callback, this);
    */
    ros::Duration(3).sleep();
    is_jointpos_controller_active_ = false;
    is_cartpos_controller_active_ = false;
        return true;
}

void QNode::joints_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joints_state_mutex_.lock();
  joints_state_ = *msg;
  joints_state_mutex_.unlock();

  // Signal the UI that there is a new joints state message
  Q_EMIT jointsStateArrived();
}

void QNode::joints_error_callback(const wam_dmp_controller::SetJointPosStampedMsgConstPtr& msg)
{
  //log(Info,std::string("joint_error arrived!" ));
  joints_error_mutex_.lock();
  joints_error_ = *msg;
  joints_error_mutex_.unlock();

  // Signal the UI that there is a joint position error message
  // only if Cartesian Position Controller is active
  //if (is_jointpos_controller_active_)
  Q_EMIT jointsErrorArrived();
}

void QNode::cart_pos_callback(const wam_dmp_controller::PoseRPYConstPtr& msg)
{
  cart_pos_mutex_.lock();
  cart_pos_ = *msg;
  cart_pos_mutex_.unlock();
  //log(Info,std::string("CART POS RECEIVED!!!!!!!! "));
  // Signal the UI that there is a new joints state message
  Q_EMIT CartPosArrived();
}

void QNode::cart_error_callback(const wam_dmp_controller::PoseRPYConstPtr& msg)
{
  cart_error_mutex_.lock();
  cart_err_ = *msg;
  cart_error_mutex_.unlock();
  //log(Info,std::string("CART ERR RECEIVED!!!!!!!! "));
  // Signal the UI that there is a new joints state message
  Q_EMIT CartErrorArrived();
}


void QNode::get_joints_state(std::vector<double>& positions)
{
  joints_state_mutex_.lock();
  positions = joints_state_.position;
  joints_state_mutex_.unlock();
}

void QNode::get_joints_error(std::vector<double>& errors)
{

  joints_error_mutex_.lock();
  errors = joints_error_.joint_pos;
  joints_error_mutex_.unlock();
}

void QNode::get_cart_pos(geometry_msgs::Vector3& trans, wam_dmp_controller::RPY& rot)
{
    cart_pos_mutex_.lock();
    trans = cart_pos_.position;
    rot = cart_pos_.orientation;
    cart_pos_mutex_.unlock();
}

void QNode::get_cart_error(geometry_msgs::Vector3& trans_err, wam_dmp_controller::RPY& rot_err)
{

    cart_error_mutex_.lock();
    trans_err = cart_err_.position;
    rot_err = cart_err_.orientation;
    cart_error_mutex_.unlock();
}

void QNode::get_trajectories_progress()
{
   get_current_cmd<wam_dmp_controller::JointPosSpline,\
                   wam_dmp_controller::JointPosSplineMsg>(progress_joint_);

  get_current_cmd<wam_dmp_controller::PoseRPYCommand,\
                  wam_dmp_controller::PoseRPYCmd>(progress_cart_);
  /*
  get_current_cmd<lwr_force_position_controllers::HybridImpedanceCommandTrajForce,\
          lwr_force_position_controllers::HybridImpedanceCommandTrajForceMsg>(progress_hybrid_force_);
  */
}

void QNode::get_progress_jointpos(double& elapsed, double& duration)
{
  elapsed = progress_joint_.elapsed_time;
  duration = progress_joint_.p2p_traj_duration;
}

void QNode::get_progress_cartpos(double& elapsed, double& duration)
{
  elapsed = progress_cart_.elapsed_time;
  duration = progress_cart_.p2p_traj_duration;
}

void QNode::run() {
    ros::Rate loop_rate(10);
    double frequency = 10;
    double step = 1.0 / 10;
    double elapsed = 0;
        int count = 0;
        while ( ros::ok() ) {

                std_msgs::String msg;
                std::stringstream ss;
        ss << "hello world " << count;
                msg.data = ss.str();
        chatter_publisher.publish(msg);
        log(Info,std::string("I sent: ")+msg.data);
                ros::spinOnce();
                loop_rate.sleep();
        elapsed += step;
        if (elapsed > 1)
        {
          /*
          ss << progress_joint_.elapsed_time;
          msg.data = ss.str();
          log(Info,std::string("elapsed time: ")+msg.data);
          ss.str("");
          ss << progress_joint_.p2p_traj_duration;
          msg.data = ss.str();
          log(Info,std::string("segment duration: ")+msg.data);
          ss.str("");
          ss << (progress_joint_.elapsed_time / progress_joint_.p2p_traj_duration);
          msg.data = ss.str();
          log(Info,std::string("current progress: ")+msg.data);
          */
          //log(QNode::LogLevel::Info, std::string("progress bar updated"));
          get_trajectories_progress();
          Q_EMIT progressDataArrived();
        }
                ++count;
        }
        std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
        logging_model.insertRows(logging_model.rowCount(),1);
        std::stringstream logging_model_msg;
        switch ( level ) {
                case(Debug) : {
                                ROS_DEBUG_STREAM(msg);
                                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
                                break;
                }
                case(Info) : {
                                ROS_INFO_STREAM(msg);
                                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                                break;
                }
                case(Warn) : {
                                ROS_WARN_STREAM(msg);
                                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                                break;
                }
                case(Error) : {
                                ROS_ERROR_STREAM(msg);
                                logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
                                break;
                }
                case(Fatal) : {
                                ROS_FATAL_STREAM(msg);
                                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
                                break;
                }
        }
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

bool QNode::get_controllers_list(std::vector<std::string>& running_list, std::vector<std::string>& stopped_list)
{
  ros::NodeHandle n;
  ros::ServiceClient client;
  controller_manager_msgs::ListControllers service;
  std::vector<controller_manager_msgs::ControllerState> controller_list;

  ros::service::waitForService("/controller_manager/list_controllers");

  client = n.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");
  if(!client.call(service))
    return false;
  controller_list = service.response.controller;

  for (std::vector<controller_manager_msgs::ControllerState>::iterator it = controller_list.begin();
   it != controller_list.end(); ++it)
    {
  if(it->state == "running")
    running_list.push_back(it->name);
  else if (it->state == "stopped")
    stopped_list.push_back(it->name);
    }

  return true;
}

bool QNode::switch_controllers(const std::string start_controller, const std::string stop_controller, bool& switch_ok)
{
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
  controller_manager_msgs::SwitchController service;
  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;
  start_controllers.push_back(start_controller);
  stop_controllers.push_back(stop_controller);
  service.request.start_controllers = start_controllers;
  service.request.stop_controllers = stop_controllers;
  service.request.strictness = 2;

  if(!client.call(service))
    return false;

  switch_ok = service.response.ok;

  return true;
}

void QNode::set_robot_namespace(std::string name)
{
  robot_namespace_ = name;
}

void QNode::get_robot_namespace(std::string& name)
{
  name = robot_namespace_;
}
}  // namespace barrett_controller_GUI
