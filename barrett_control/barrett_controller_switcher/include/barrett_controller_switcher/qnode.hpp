/**
 * @file /include/barrett_controller_switcher/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef barrett_controller_switcher_QNODE_HPP_
#define barrett_controller_switcher_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Float64MultiArray.h>
#include <wam_dmp_controller/SetJointPos.h>
#include <wam_dmp_controller/GetJointPos.h>
#include <wam_dmp_controller/SetJointPosMsg.h>
#include <wam_dmp_controller/SetJointGains.h>
#include <wam_dmp_controller/GetJointGains.h>
#include <wam_dmp_controller/SetJointGainsMsg.h>
#include <wam_dmp_controller/GoHome.h>
#include <wam_dmp_controller/GoHomeMsg.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ControllerState.h>



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace barrett_controller_switcher {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

    template <class ServiceType, class ServiceMessageType>
    bool set_command(ServiceMessageType command, ServiceMessageType& response, bool& state);
    template <class ServiceType, class ServiceMessageType>
    bool get_current_cmd(ServiceMessageType& current_command);
    bool get_controllers_list(std::vector<std::string>& running_list, std::vector<std::string>& stopped_list);
    bool switch_controllers(const std::string start_controller, const std::string stop_controller, bool& switch_ok);
    void set_robot_namespace(std::string name);
Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

    std::string robot_namespace_;
};

template <class ServiceType, class ServiceMessageType>
bool QNode::set_command(ServiceMessageType command, ServiceMessageType& response, bool& state)
{
  ros::NodeHandle n;
  ros::ServiceClient client;
  ServiceType service;
  std::string service_name;

  // Choose the service name depending on the ServiceType type

  if(std::is_same<ServiceType, wam_dmp_controller::SetJointPos>::value)
    service_name = "/" + robot_namespace_ + "/joint_space_controller/set_joint_pos";
  else if(std::is_same<ServiceType, wam_dmp_controller::SetJointGains>::value)
    service_name = "/" + robot_namespace_ + "/joint_space_controller/set_gains";
  else if (std::is_same<ServiceType, wam_dmp_controller::GoHome>::value)
    service_name = "/" + robot_namespace_ + "/joint_space_controller/go_home";
  /*
  else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommandTrajForce>::value)
    service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/set_hybrid_traj_force_cmd";
  else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommandGains>::value)
    service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/set_hybrid_gains_cmd";
  else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceSwitchForcePos>::value)
    service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/switch_force_position_z";
   */
  client = n.serviceClient<ServiceType>(service_name);

  service.request.command = command;

  if (client.call(service))
   {
      //response = service.response.command;
      state = service.response.state;
      return true;
   }
  else
    return false;
}

template <class ServiceType, class ServiceMessageType>
bool QNode::get_current_cmd(ServiceMessageType& current_command)
{
  ros::NodeHandle n;
  ros::ServiceClient client;
  ServiceType service;
  std::string service_name;
  double outcome;

  // Choose the service name depending on the ServiceType type

  if(std::is_same<ServiceType, wam_dmp_controller::GetJointPos>::value)
    service_name = "/" + robot_namespace_ + "/joint_space_controller/get_joint_pos";
  else if(std::is_same<ServiceType, wam_dmp_controller::GetJointGains>::value)
    service_name = "/" + robot_namespace_ + "/joint_space_controller/get_gains";
  /*
  else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommandTrajPos>::value)
    service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/get_hybrid_traj_pos_cmd";
  else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommandTrajForce>::value)
    service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/get_hybrid_traj_force_cmd";
  else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommandGains>::value)
    service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/get_hybrid_gains_cmd";
  */
  client = n.serviceClient<ServiceType>(service_name);

  outcome = client.call(service);

  if (outcome)
    current_command = service.response.command;

  return outcome;
}

}  // namespace barrett_controller_switcher

#endif /* barrett_controller_switcher_QNODE_HPP_ */
