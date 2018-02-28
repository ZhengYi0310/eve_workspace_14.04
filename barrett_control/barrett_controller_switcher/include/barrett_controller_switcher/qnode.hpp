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
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Float64MultiArray.h>


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
    bool set_command(ServiceMessageType command, ServiceMessageType& response);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;
};

template <class ServiceType, class ServiceMessageType>
bool QNode::set_command(ServiceMessageType command, ServiceMessageType& response)
{
  ros::NodeHandle n;
  ros::ServiceClient client;
  ServiceType service;
  std::string service_name;

  // Choose the service name depending on the ServiceType type
  /*
  if(std::is_same<ServiceType, lwr_force_position_controllers::CartesianPositionCommandTraj>::value)
    service_name = "/" + robot_namespace_ + "/cartesian_position_controller/set_cartpos_traj_cmd";
  else if(std::is_same<ServiceType, lwr_force_position_controllers::CartesianPositionCommandGains>::value)
    service_name = "/" + robot_namespace_ + "/cartesian_position_controller/set_cartpos_gains_cmd";
  else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommandTrajPos>::value)
    service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/set_hybrid_traj_pos_cmd";
  else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommandTrajForce>::value)
    service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/set_hybrid_traj_force_cmd";
  else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommandGains>::value)
    service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/set_hybrid_gains_cmd";
  else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceSwitchForcePos>::value)
    service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/switch_force_position_z";
  client = n.serviceClient<ServiceType>(service_name);
  */
  service.request.command = command;

  if (client.call(service))
    {
  response = service.response.command;
  return true;
    }
  else
    return false;
}

}  // namespace barrett_controller_switcher

#endif /* barrett_controller_switcher_QNODE_HPP_ */
