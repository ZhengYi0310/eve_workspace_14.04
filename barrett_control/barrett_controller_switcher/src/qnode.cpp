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
#include "../include/barrett_controller_switcher/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace barrett_controller_switcher {

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
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    set_robot_namespace("barrett_hw");
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
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

}  // namespace barrett_controller_switcher
