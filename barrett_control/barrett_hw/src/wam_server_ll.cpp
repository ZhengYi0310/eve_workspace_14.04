#include <ros/ros.h>
#include <native/task.h>
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
#include <barrett/systems.h>
#include <barrett/bus/can_socket.h>
#include <barrett/products/product_manager.h>

#include <barrett_model/semi_absolute_joint_interface.h>

#include <terse_roscpp/param.h>

#include <urdf/model.h>

#include <stdexcept>

bool g_quit = false;

void quitRequested(int sig) {
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
    void stop();
    void cleanup() { }

    // Wait for all devices to become active
    bool wait_for_mode(
        barrett::SafetyModule::SafetyMode mode,
        ros::Duration timeout = ros::Duration(60.0), 
        ros::Duration poll_duration = ros::Duration(0.1));

    void set_mode(
        barrett::SafetyModule::SafetyMode mode);

    // State structure for a Wam
    // This provides storage for the joint handles
    template<size_t DOF>
      struct WamDevice 
      {
        // Low-level interface
        boost::shared_ptr<barrett::LowLevelWam<DOF> > interface;

        // Configuration
        std::vector<std::string> joint_names;
        Eigen::Matrix<double,DOF,1> 
          resolver_ranges,
          effort_limits,
          velocity_limits;

        // State
        Eigen::Matrix<double,DOF,1> 
          joint_positions,
          joint_velocities,
          joint_effort_cmds,
          joint_offsets,
          resolver_angles,
          calibration_burn_offsets;

        Eigen::Matrix<int,DOF,1> calibrated_joints;

        void set_zero() {
          joint_positions.setZero();
          joint_velocities.setZero();
          joint_effort_cmds.setZero();
          joint_offsets.setZero();
          resolver_angles.setZero();
          calibration_burn_offsets.setZero();
        }

      };

    // State structure for a Hand
    struct HandDevice 
    {
      // Low-level interface
      boost::shared_ptr<barrett::Hand> interface;

      // Configuration
      std::vector<std::string> joint_names;
      Eigen::Vector4d resolver_ranges;

      // State
      Eigen::Vector4d
        joint_positions,
        joint_velocities,
        joint_effort_cmds,
        joint_offsets,
        resolver_angles,
        calibration_burn_offsets;

      Eigen::Vector4i calibrated_joints;
    };

    // Typedefs to make the world happier
    typedef WamDevice<4> WamDevice4;
    typedef WamDevice<7> WamDevice7;
    typedef std::map<std::string, boost::shared_ptr<barrett::ProductManager> >  ManagerMap;
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

    // ros-controls interface
    hardware_interface::JointStateInterface state_interface_;
    hardware_interface::EffortJointInterface effort_interface_;
    barrett_model::SemiAbsoluteJointInterface semi_absolute_interface_;

    // Vectors of various barrett structures
    ManagerMap barrett_managers_;
    Wam4Map wam4s_;
    Wam7Map wam7s_;
    HandMap hands_;

  protected:

    template <size_t DOF>
      Eigen::Matrix<double,DOF,1>
      compute_resolver_ranges(boost::shared_ptr<barrett::LowLevelWam<DOF> > wam);

    template <size_t DOF>
      boost::shared_ptr<BarrettHW::WamDevice<DOF> > 
      configure_wam(
          ros::NodeHandle product_nh,
          boost::shared_ptr<barrett::ProductManager> barrett_manager, 
          const libconfig::Setting &wam_config); 

    template <size_t DOF>
      bool
      read_wam(
          const ros::Time time, 
          const ros::Duration period,
          boost::shared_ptr<BarrettHW::WamDevice<DOF> > device);

    template <size_t DOF>
      void 
      write_wam(
          const ros::Time time, 
          const ros::Duration period,
          boost::shared_ptr<BarrettHW::WamDevice<DOF> > device);
  };

  BarrettHW::BarrettHW(ros::NodeHandle nh) :
    nh_(nh),
    configured_(false),
    calibrated_(false)
  {
    // TODO: Determine pre-existing calibration from ROS parameter server
  }

  bool BarrettHW::configure() 
  {
    using namespace terse_roscpp;
    std::vector<std::string> product_names;

    // Get URDF
    std::string urdf_str;
    param::require(nh_, "robot_description_yi", urdf_str, "The URDF for this barrett system.");
    urdf_model_.initString(urdf_str);

    // Load parameters
    param::require(nh_,"product_names",product_names, "The unique barrett product names.");

    for(std::vector<std::string>::const_iterator it = product_names.begin();
        it != product_names.end();
        ++it) 
    {
      const std::string &product_name = *it;
      ros::NodeHandle product_nh(nh_,"products/"+product_name);

      // Determine the bus for this product
      std::string bus_name;
      param::require(product_nh,"bus",bus_name, "Bus name.");

      // Get the barrett product manager
      boost::shared_ptr<barrett::ProductManager> barrett_manager;

      // Create the product manager if it doesn't exist
      if(barrett_managers_.find(bus_name) == barrett_managers_.end()) {
        // Determine the bus information 
        int bus_port;
        param::require(nh_,"busses/"+bus_name+"/port", bus_port, "Bus port [0-n].");
        std::string config_path;
        bool config_path_found = param::get(nh_,"busses/"+bus_name+"/config", config_path, "Path to a libbarrett config file.");

        // Create a new bus/manager
        //boost::shared_ptr<barrett::bus::CANSocket> canbus(new barrett::bus::CANSocket(bus_port));
        barrett_manager.reset(
            new barrett::ProductManager(config_path_found ? config_path.c_str() : NULL /* Use defailt config */));
        barrett_managers_[bus_name] = barrett_manager;
      } 
        else {
                // Use the existing bus/manager
                barrett_manager = barrett_managers_[bus_name];
            }
        //ROS_INFO("Product manager contructed!");
        
      // Get the product information
      std::string product_type;
      param::require(product_nh,"type",product_type, "Barrett product type [wam,bhand].");

      // Add products
      if(product_type == "wam") {
        // Get the configuration for this type of arm
        const libconfig::Setting& wam_config = barrett_manager->getConfig().lookup(barrett_manager->getWamDefaultConfigPath());

        // Construct and store the wam interface
        if(barrett_manager->foundWam4()) { 
          wam4s_[product_name] = this->configure_wam<4>(product_nh, barrett_manager, wam_config);
        } else if(barrett_manager->foundWam7()) {
          wam7s_[product_name] = this->configure_wam<7>(product_nh, barrett_manager, wam_config);
        } else {
          ROS_ERROR("Could not find WAM on bus!"); 
          continue; 
        }
      } else if(product_type == "hand") {
        ROS_ERROR_STREAM("Look ma, no hands!");
        continue;
      } else {
        ROS_ERROR_STREAM("Unknown Barrett product type: "+product_type);
        continue;
      }
    }

    // TODO: Set up gravity compensator
    // Write kdl_ros_integration package which handles kdl/urdf/ros_control? interfaces

    // Register ros-controls interfaces
    this->registerInterface(&state_interface_);
    this->registerInterface(&effort_interface_);
    //this->registerInterface(&semi_absolute_interface_);

    // Set configured flag
    configured_ = true;

    return true;
  }

  template <size_t DOF>
    Eigen::Matrix<double,DOF,1>
    BarrettHW::compute_resolver_ranges(boost::shared_ptr<barrett::LowLevelWam<DOF> > wam) 
    {
      Eigen::MatrixXd m_to_j_pos = wam->getMotorToJointPositionTransform();  
      return (m_to_j_pos.diagonal().array() * 2.0*M_PI).cwiseAbs().matrix();
    }

  template<size_t DOF>
    boost::shared_ptr<BarrettHW::WamDevice<DOF> > 
    BarrettHW::configure_wam(
        ros::NodeHandle product_nh,
        boost::shared_ptr<barrett::ProductManager> barrett_manager, 
        const libconfig::Setting &wam_config) 
    {
      using namespace terse_roscpp;

      // Construct a new wam device (interface and state storage)
      boost::shared_ptr<BarrettHW::WamDevice<DOF> > wam_device(new BarrettHW::WamDevice<DOF>());

      // Get the wam pucks
      std::vector<barrett::Puck*> wam_pucks = barrett_manager->getWamPucks();
      wam_pucks.resize(DOF);

      // Construct a low-level wam
      wam_device->interface.reset(
          new barrett::LowLevelWam<DOF>(
            wam_pucks, 
            barrett_manager->getSafetyModule(), 
            wam_config["low_level"]));

      // Initialize resolver ranges
      wam_device->resolver_ranges = this->compute_resolver_ranges<DOF>(wam_device->interface);

      // Get URDF links starting at product root link
      std::string tip_joint_name;
      param::require(product_nh,"tip_joint",tip_joint_name, "WAM tip joint name in URDF.");
      boost::shared_ptr<const urdf::Joint> joint = urdf_model_.getJoint(tip_joint_name);

      // Resize joint names
      wam_device->joint_names.resize(DOF);

      // Create joint handles starting at the tip
      for(int i=DOF-1; i>=0; i--) {
        // While the joint has been handled or the joint type isn't revolute
        while(std::find(wam_device->joint_names.begin(), wam_device->joint_names.end(),joint->name) != wam_device->joint_names.end() 
            || joint->type != urdf::Joint::REVOLUTE)
        {
          // Get the next joint
          joint = urdf_model_.getLink(joint->parent_link_name)->parent_joint;
          // Make sure we didn't run out of links
          if(!joint.get()) {
            ROS_ERROR_STREAM("Ran out of joints while parsing URDF starting at joint: "<<tip_joint_name);
            throw std::runtime_error("Ran out of joints.");
          }
        }

        ROS_INFO("parsing joint %i with name >>%s<<!", int(i) + 1, (joint->name).c_str());
    
        // Store the joint name
        wam_device->joint_names[i] = joint->name;
        wam_device->effort_limits(i) = joint->limits->effort;
        wam_device->velocity_limits(i) = joint->limits->velocity;

        // Joint State Handle
        hardware_interface::JointStateHandle state_handle(joint->name,
            &wam_device->joint_positions(i),
            &wam_device->joint_velocities(i),
            &wam_device->joint_effort_cmds(i));
        state_interface_.registerHandle(state_handle);

        // Effort Command Handle
        effort_interface_.registerHandle(
            hardware_interface::JointHandle(
              state_interface_.getHandle(joint->name),
              &wam_device->joint_effort_cmds(i)));

        // Transmission / Calibration handle
        semi_absolute_interface_.registerJoint(
            effort_interface_.getHandle(joint->name),
            wam_device->resolver_ranges(i),
            &wam_device->resolver_angles(i),
            &wam_device->joint_offsets(i),
            &wam_device->calibrated_joints(i));

      }

      return wam_device;
    }

  bool BarrettHW::start()
  {
    // Guard on configured
    if(!configured_) {
      ROS_ERROR("Barrett hardware must be configured before it can be started.");
      return false;
    }

    // Zero the state 
    for(Wam4Map::iterator it = wam4s_.begin(); it != wam4s_.end(); ++it) {
      it->second->set_zero();
    }
    for(Wam7Map::iterator it = wam7s_.begin(); it != wam7s_.end(); ++it) {
      it->second->set_zero();
    }

    // Wait for the system to become active
    this->wait_for_mode(barrett::SafetyModule::ACTIVE);
    ROS_INFO("hardware interface started!");

    return true;
  }

  void BarrettHW::stop()
  {
    // Set the mode to IDLE
    this->set_mode(barrett::SafetyModule::IDLE);
    // Wait for the system to become active
    this->wait_for_mode(barrett::SafetyModule::IDLE);
  }

  bool BarrettHW::read(const ros::Time time, const ros::Duration period)
  {
    // Iterate over all devices
    for(Wam4Map::iterator it = wam4s_.begin(); it != wam4s_.end(); ++it) {
      this->read_wam(time, period, it->second);
    }
    for(Wam7Map::iterator it = wam7s_.begin(); it != wam7s_.end(); ++it) {
      this->read_wam(time, period, it->second);
    }

    return true;
  }

  void BarrettHW::write(const ros::Time time, const ros::Duration period)
  {
    // Iterate over all devices
    for(Wam4Map::iterator it = wam4s_.begin(); it != wam4s_.end(); ++it) {
      this->write_wam(time, period, it->second);
    }
    for(Wam7Map::iterator it = wam7s_.begin(); it != wam7s_.end(); ++it) {
      this->write_wam(time, period, it->second);
    }
  }

  template <size_t DOF>
    bool BarrettHW::read_wam(
        const ros::Time time, 
        const ros::Duration period,
        boost::shared_ptr<BarrettHW::WamDevice<DOF> > device)
    {
      // Poll the hardware
      try {
        device->interface->update();
      } catch (const std::runtime_error& e) {
        if (device->interface->getSafetyModule() != NULL  &&
            device->interface->getSafetyModule()->getMode(true) == barrett::SafetyModule::ESTOP) 
        {
          ROS_ERROR_STREAM("systems::LowLevelWamWrapper::Source::operate(): E-stop! Cannot communicate with Pucks.");
          return false;
        } else {
          throw;
        }
      }

      // Get raw state
      Eigen::Matrix<double,DOF,1> raw_positions = device->interface->getJointPositions();
      Eigen::Matrix<double,DOF,1> raw_velocities = device->interface->getJointVelocities();

      // Smooth velocity 
      // TODO: parameterize time constant
      for(size_t i=0; i<DOF; i++) {
        device->joint_velocities(i) = filters::exponentialSmoothing(
            raw_velocities(i),
            device->joint_velocities(i),
            0.5);
      }

      // Store position
      device->joint_positions = raw_positions;

      // Read resolver angles
      std::vector<barrett::Puck*> pucks = device->interface->getPucks();	
      for(size_t i=0; i<pucks.size(); i++) {
        device->resolver_angles(i) = pucks[i]->getProperty(barrett::Puck::MECH);
      }

      return true;
    }

  template <size_t DOF>
    void BarrettHW::write_wam(
        const ros::Time time, 
        const ros::Duration period,
        boost::shared_ptr<BarrettHW::WamDevice<DOF> > device)
    {
      static int warning = 0;

      for(size_t i=0; i<DOF; i++) {
        if(std::abs(device->joint_effort_cmds(i)) > device->effort_limits[i]) {
          if(warning++ > 1000) {
            ROS_WARN_STREAM("Commanded torque ("<<device->joint_effort_cmds(i)<<") of joint ("<<i<<") exceeded safety limits! They have been truncated to: +/- "<<device->effort_limits[i]);
            warning = 0;
          }
          // Truncate this joint torque
          device->joint_effort_cmds(i) = std::max(
              std::min(device->joint_effort_cmds(i), device->effort_limits[i]),
              -1.0*device->effort_limits[i]);
        }
      }

      // Set the torques
      device->interface->setTorques(device->joint_effort_cmds);

      // If not calibrated, servo estimated position to calibration position
      static int calib_decimate = 0;
      if(!calibrated_ && calib_decimate++ > 0) {
        calib_decimate = 0;

        // Check if each joint is calibrated, if
        bool all_joints_calibrated = true;
        for(size_t i=0; i<DOF; i++) {
          if(!all_joints_calibrated) {
            device->calibration_burn_offsets = device->joint_positions;
          }
          all_joints_calibrated = all_joints_calibrated && device->calibrated_joints[i] == 1;
        }

        if(all_joints_calibrated) {

#if 0
          // Setting the positions cannot violate the velocity limits
          // We need to update them incrementally
          double minimum_time = calibration_burn_offsets_.cwiseQuotient(velocity_limits_).cwiseAbs().maxCoeff();
          double step = std::min(1.0,std::max(period.toSec()/minimum_time,0.0));

          calibration_burn_offsets_ -= (step)*calibration_burn_offsets_;
          joint_offsets_.data += (step)*calibration_burn_offsets_;

          static int decimate =0;
          if(decimate++ > 100) {
            ROS_INFO_STREAM("Adjusting offset by: "<<step<<" minimum time: "<<minimum_time);
            decimate = 0;
          }
#endif

          // Assign the positions to the current robot configuration
          device->calibration_burn_offsets.setZero();
          device->interface->definePosition(device->calibration_burn_offsets);

          //if(std::abs(step-1.0) < 1E-4) {
          calibrated_ = true;
          //}

          ROS_INFO("Zeroed joints.");

        }
      }
    }

  bool BarrettHW::wait_for_mode(
      barrett::SafetyModule::SafetyMode mode,
      ros::Duration timeout, 
      ros::Duration poll_duration) 
  {
    ros::Time polling_start_time = ros::Time::now();
    ros::Rate poll_rate(1.0/poll_duration.toSec());

    for(ManagerMap::iterator it = barrett_managers_.begin(); 
        it != barrett_managers_.end();
        ++it) 
    {
      while( ros::ok() 
          && (ros::Time::now() - polling_start_time < timeout)
          && (it->second->getSafetyModule()->getMode() != mode)) 
      {
        poll_rate.sleep();
      }
    }
    return (ros::Time::now() - polling_start_time < timeout);
  }

  void BarrettHW::set_mode(
      barrett::SafetyModule::SafetyMode mode)
  {
    for(ManagerMap::iterator it = barrett_managers_.begin(); 
        it != barrett_managers_.end();
        ++it) 
    {
      it->second->getSafetyModule()->setMode(mode);
    }
  }

}

int main( int argc, char** argv ){

  // Set up real-time task
  mlockall(MCL_CURRENT | MCL_FUTURE);
  RT_TASK task;
  rt_task_shadow( &task, "GroupWAM", 99, 0 );

  // Initialize ROS
  ros::init(argc, argv, "wam_server", ros::init_options::NoSigintHandler);

  // Add custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // Construct the wam structure
  ros::NodeHandle barrett_nh("barrett");

  barrett_hw::BarrettHW barrett_robot(barrett_nh);
  barrett_robot.configure();
  barrett_robot.start();


  // Timer variables
  struct timespec ts = {0,0};

  if(clock_gettime(CLOCK_REALTIME, &ts) != 0) {
    ROS_FATAL("Failed to poll realtime clock!");
  }

  ros::Time 
    last(ts.tv_sec, ts.tv_nsec),
    now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  realtime_tools::RealtimePublisher<std_msgs::Duration> publisher(barrett_nh, "loop_rate", 2);

  bool wam_ok = false;
  while(!g_quit && !wam_ok) {
    if(!barrett_robot.configure()) {
      ROS_ERROR("Could not configure WAM!");
    } else if(!barrett_robot.start()) {
      ROS_ERROR("Could not start WAM!");
    } else {
      ros::Duration(1.0).sleep();

      if(!barrett_robot.read(now, period)) {
        ROS_ERROR("Could not read from WAM!");
      } else {
        wam_ok = true;
      }
    }

    ros::Duration(1.0).sleep();
  }

  // Construct the controller manager
  ros::NodeHandle nh;
  controller_manager::ControllerManager manager(&barrett_robot, nh);

  uint32_t count = 0;

  // Run as fast as possible
  while( !g_quit ) {
    // Get the time / period
    if (!clock_gettime(CLOCK_REALTIME, &ts)) {
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    } else {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    } 

    // Read the state from the WAM
    if(!barrett_robot.read(now, period)) {
      g_quit=true;
      break;
    }

    // Update the controllers
    manager.update(now, period);

    // Write the command to the WAM
    barrett_robot.write(now, period);

    if(count++ > 1000) {
      if(publisher.trylock()) {
        count = 0;
        publisher.msg_.data = period;
        publisher.unlockAndPublish();
      }
    }
  }

  publisher.stop();

  std::cerr<<"Stpping spinner..."<<std::endl;
  spinner.stop();

  std::cerr<<"Stopping WAM..."<<std::endl;
  barrett_robot.stop();

  std::cerr<<"Cleaning up WAM..."<<std::endl;
  barrett_robot.cleanup();

  std::cerr<<"Poka!"<<std::endl;

  return 0;
}

