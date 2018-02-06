/*************************************************************************
	> File Name: wam_server.cpp
	> Author: 
	> Mail: 
	> Created Time: Thu 01 Jun 2017 01:44:02 PM PDT
 ************************************************************************/

#include <iostream>
#include <barrett_hw/wam_server.h>

namespace barrett_hw
{
    BarrettHW::BarrettHW(ros::NodeHandle nh) :
      nh_(nh),
      configured_(false),
      calibrated_(false)
    {
        //TODO: Determine pre-existing calibration from ROS param server
    }

    bool BarrettHW::configure()
    {
        using namespace terse_cpp;
        std::vector<std::string> product_names;

        // Get The URDF Model of Wam 
        std::string urdf_str;
        param::require(nh_, "robot_description", urdf_str, "The URDF for the Barrett Wam arm.")
        urdf_model_.initString(urdf_str);

        // Load Parameters 
        param::require(nh_, "product_names", product_names, "The unique barrett product names.");

        for (std::vector<std::string>::const_iterator it = product_names.begin(), it != product_names.end(); it++)
        {
            const std::string &product_name = *it;
            ros::NodeHandle product_nh(nh_, "product_names/"+product_name);

            // Determine the bus for this product 
            std::String bus_name;
            param::require(product_nh, "bus", bus_name, "Bus name.");

            // Get the barrett product manager 
            boost::shared_ptr<barrett::ProductManager> barrett_manager;

            // Create the product manager if it doesn't exist 
            if (barrett_managers_.find(bus_name) == barrett_managers_.end())
            {
                /**
                // Determine the bus information 
                int bus_port;
                param::require(product_nh, "busses/" + bus_name + "port", bus_port, "Bus port [0-n].");
                */
                std::string config_path;
                bool config_path_found = param::get(product_nh, "busses/" + bus_name +"/config", config_path, "Path to a libbarrett configuration file.");

                // Create a new manager 
                barrett_manager.reset(new barrett::ProductManager(config_path_found ? config_path.c_str() : NULL /* USE DEFAULT CONFIG OTHERWISE */));
            
                barrett_managers_[bus_name] = barrett_manager;
            }

            else 
            {
                barrett_manager = barrett_managers_[bus_name];
            }

            // Get the product information 
            std::string product_type;
            param::require(product_nh, "type", product_type, "Barrett product type [wam, bhand].");

            // Add products 
            if (product_type == "wam")
            {
                // Get the configuration for this type of arm 
                const libconfig::Setting& wam_config = barrett_manager->getConfig().lookup(config_path.c_str());

                // Construct and store the wam interface
                barrett_manager->waitForWam() //prompt on zeroing 
                barrett_manager->wakeAllPucks();

                if (barrett_manager->foundWam4())
                {
                    wam4s_[product_name] = this->configure_wam<4>(product_nh, barrett_manager, wam_config);
                }
                else if (barrett_manager->foundWam7())
                {
                    wam7s_[product_name] = this->configure_wam<7>(product_nh, barrett_manager, wam_config);
                }
                else
                {
                    ROS_ERROR_STREAM("Cound not find Wam on bus " << bus_port << "!");
                    continue;
                }
            }
            else if(product_type == "hand")
            {
                //bring up the biotac from here Maybe !!!!!!!!!!!!!!!!!**********************
                ROS_ERROR_STREAM("Look ma, no hands!");
                continue;
            }
            else 
            {
                ROS_ERROR_STREAM("Unknown Barret Product types: " << product_type);
                continue;
            }
        }

        // Register ros-controllers interfaces 
        this->registerInterface(&state_interface_);
        this->registerInterface(&effort_interface_);
        this->registerInterface(&semi_absolute_interface_);

        // Set the configured flag 
        configured_ = true;

        return true 
    }

    template <size_t DOF>
    Eigen::Matrix<double, DOF, 1> BarrettHW::compute_resolver_ranges(barrett::LowLevelWam<DOF>& wam)
    {
        Eigen::MatrixXd m_to_j_pos = wam.getMotorToJointPositionTransform();
        return (m_to_j_pos.diagonal().array() * 2.0 * M_PI).cwiseAbs().matrix();
    }

    template<size_t DOF>
    boost::shared_ptr<BarrettHW::WamDevice<DOF> > BarrettHW::configure_wam(ros::NodeHandle product_nh, boost::shared_ptr<barrett::ProductManager> barrett_manager, const libconfig::Setting &wam_config)
    {
        using namespace terse_roscpp;

        // Construct a new wam device (interface and state storage)
        boost::shared_ptr<BarrettHW::WamDevice<DOF> > wam_device(new BarrettHW::WamDevice<DOF>());

        //******************************
        // Get the Wam interface and the LowLevelWam interface
        if (DOF == 4)
        {
            // wait for shift-activate 
            wam_device->Wam = barrett_manager->getWam4(true);
        }
        else if (DOF==7)
        {
            //wait for shift-activate 
            wam_device->Wam = barrett_manager->getWam7(true);
        }
        else
        {
            ROS_ERROR_STREAM("Invalid DOF: " << DOF << "!");
        }
        wam_device->interface = wam_device->Wam->getLowLevelWam();
        //***********************
        
        wam_device->resolver_ranges = this->compute_resolver_ranges<DOF>(wam_device->interface);

        // Get URDF links starting at the product tip link 
        std::string tip_joint_name;
        param::require(product_nh, "tip_joint", tip_joint_name, "WAM tip joint in URDF.");
        boost::shared_ptr<const urdf::Joint> joint = urdf_model_.getJoint(tip_joint_name);

        // Resize joint names 
        wam_device->joint_names.resize(DOF);

        // Create joint handles starting at the tip 
        for (size_t i = DOF-1; i >=0; i--)
        {
            while (std::find(wam_device->joint_names.begin(), wam_device->joint_names.end(), joint->name) != wam_device->joint_names.end() || joint->type != urdf::Joint::REVOLUTE)
            {
                // Get the next joint 
                joint = urdf_model_.getLink(joint->parent_link_name)->parent_joint;
                // Make sure we didn't run out of links 
                if (!joint.get())
                {
                    ROS_ERROR_STREAM("Run out of joints while parsing URDF starting at joint: " << tip_joint_name);
                    throw std::runtime_error("Run out of joints.");
                }
            }

            // Store the joint name 
            wam_device->joint_names[i] = joint->name;
            wam_device->effor_limits(i) = joint->limits->effort;
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
            semi_absolute_interface_.registerHandle(
                effort_interface_.getHandle(joint->name),
                wam_device->resolver_ranges(i),
                &wam_device->resolver_angles(i),
                &wam_device->joint_offsets(i),
                &wam_device->calibrated_joints(i));
        }

        barrett_manager->getExecutionManager()->startManaging(ramp); //starting ramp manager

        ROS_INFO("%zu-DOF WAM Left.");
        jp_home = wam_device->Wam->getJointPositions();

        if (barrett_manager->foundHand()) //Does the following only if a BarrettHand is found 
        {
            boost::shared_ptr<BarrettHW::HandDevice> hand_device(new BarrettHW::HandDevice())       
            hand_device->interface = barrett_manager->getHand();
            
            // Asjust the torque limits for BarrentHands movements at extents 
            barrett_manager->getSafetyModule()->setTorqueLimit(3.0);

            // Move j3 in order to give room for hand initialization 
            jp_type jp_init = wam_device->Wam->getJointPositions();
            jp_init[3] -= 0.35;
            usleep(500000);
            wam_device->Wam->moveTo(jp_init);

            usleep(500000);
            hand_device->interface->initialize();
            hand_device->interface->update();

            //ROS_INFO("Closing the BarrenttHand Grasp");
            hand_device->close(barrett::Hand::GRASP, false);

            wam_device->hand_device = hand_device;
        }

        wam_device->Wam->gravityCompensate(true);

        return wam_device;
    }

    bool BarrettHW::start()
    {
        // Guard on the configured 
        if (!configured_)
        {
            ROS_ERROR("Barrett hardware must be configured before it can be started.");
            return false;
        }

        // Zero the state 
        for (Wam4Map::iterator it = wam4s_.begin(); it != wam4s_.end(); it++)
        {
            it->second->set_zero();
        }
        for (Wam7Map::iterator it = wam7s_.begin(); it != wam7s_.end(); it++)
        {
            it->second->set_zero();
        }

        // Wait for the system to become ACTIVE 
        this->wait_for_mode(barrett::SafetyModule::ACTIVE);

        return true;
    }

    void BarrettHW::stop()
    {
        // Set the mode to IDLE 
        this->set_mode(barrett::SafetyModule::IDLE);
        // Wait for the system to become IDLE
        this->wait_for_mode(barrett::SafetyModule::IDLE);
    }

    bool BarrettHW::read(const ros:Time time, const ros::Duration period)
    {
        // Iterate over all deives 
        for (Wam4Map::iterator it = wam4s_.begin(); it != wam4s_.end(); it++)
        {
            this->read_wam(time, period, it->second);
        }
        for (Wam7Map::iterator it = wam7s_.begin(); it != wam7s_.end(); it++)
        {
            this->read_wam(time, period, it->second);
        }

        return true;
    }

    void BarrettHW::write(const ros::Time time, const ros::Duration period)
    {
        // Iterate over all devices 
        for (Wam4Map::iterator it = wam4s_.begin(); it != wam4s_.end(); it++)
        {
            this->write_wam(time, period, it->second);
        }
        for (Wam7Map::iterator it = wam7s_.begin(); it != wam7s_.end(); it++)
        {
            this->write_wam(time, period, it->second);
        }
    }

    template <size_t DOF>
    bool BarrettHW::read_wam(const ros::Time time, const ros::Duration period, boost::shared_ptr<BarrettHW::WamDevice<DOF> > device)
    {
        // Poll the hardware
        try 
        {
            &(device->interface)->update();
        }
        catch (const std::runtime_error &e)
        {
            if (device->interface->getSafetyModule() != NULL && device->interface->getSafetyModule()->getMode(true) == barrett::SafetyModule::ESTOP)
            {
                ROS_ERROR_STREAM("systems::LowLevelWamWrapper::Source::operate(): E-stop! Cannot communicate with Pucks!");
                return false;
            }
            else 
            {
                throw;
            }
        }

        // Get raw state 
        Eigen::Matrix<double, DOF, 1> raw_positions = &(device->interface).getJointPositions();
        Eigen::Matrix<double, DOF, 1> raw_velocities = &(device->interface).getJointVelocities();

        // Smooth velocity 
        // TODO: parameterrize time constant 
        for (size_t i = 0; i < DOF; i++)
        {
            device->joint_velocities(i) = filters::exponentialSmoothing(raw_velocities(i), deives->joint_velocities(i), 0.8);
        }

        // Store position 
        device->joint_positions = raw_positions;

        // Read resolver angles 
        std::vector<barrett::Puck*> pucks = &(device->interface)->getPucks();
        for (size_t i = 0; i < pucks.size(); i++)
        {
            device->resolver_angles(i) = pucks[i]->getProperty(barrett::Puck::MECH);
        }

        return true;
    }

    template <size_t DOF>
    void BarrettHW::write_wam(const ros::Time time, const ros::Duration period, boost::shared_ptr<BarrettHW::WamDevice<DOF> > device)
    {
        static int warning = 0;

        for (size_t i = 0; i < DOF; i++)
        {
            if (std::abs(device->joint_effort_cmds(i)) > device->effor_limits(i))
            {
                ROS_WARN_STREAM("Commanded torque (" << device->joint_effort_cmds(i) << ") of joint (" << i << ") exceeds safety limits! They have been truncated to: +/- " << device->effor_limits(i));
            
            // Truncate this joint torque 
            device->joint_effort_cmds(i) = std::max(std::min(device->joint_effort_cmds(i), device->effor_limits(i)), -1.0*device->effort_limits(i));
            }
        }

        // Set the torques 
        device->interface->setTorques(device->joint_effort_cmds);

        //*******************
        // Stuff to do about calibration 
        //******************* 
    }

    bool BarrettHW::wait_for_mode(barrett::SafetyModule mode, ros::Duration timeout, ros::Duration poll_duration)
    {
        ros::Time polling_start_time = ros::Time::now();
        ros::Rate poll_rate(1.0 / poll_duration.toSec());

        for (ManagerMap::iterator it = barrett_managers_.begin(); it != barrett_managers_.end(); it++)
        {
            while (ros::ok() && (ros::Time::now() - polling_start_time < timeout) && (it->second->getSafetyModule->getMode() != mode))
            {
                ROS_INFO_STREAM("Still waiting for mode to be set.");
                poll_rate.sleep();
            }
        }

        return (ros::Time::now() - polling_start_time < timeout);
    }

    void BarrettHW::set_mode(barrett::SafetyModule::SafetyMode mode)
    {
        for (ManagerMap::iterator it = barrett_managers_.begin(); it != barrett_managers_.end(); it++)
        {
            it->second->getSafetyModule()->setMode(mode);
        }
    }
}

int main (int argc, char** argv)
{
    // Set up real time task 
    mlockall(MCL_CURRENT | MCL_FUTURE);
    RT_TASK task;
    rt_task_shadow(&task, "GroupWAM", 99, 0);

    // initialize ROS 
    ros::init(argc, argv, "wam_server", ros::init_options::NoSigintHandler);

    // Add custom signal handlers 
    signal(SIGTERM, quitRequested);
    signal(SIGINT, quitRequested);
    signal(SIGHUP, quitRequested);

    // Construct the wam structure 
    ros::NodeHandle barrett_hw("barrett");
    barrett_hw::BarrettHW barrett_robot(barrett_nh);
    barrett_robot.configure();
    barrett_robot.start();

    // Timer variables 
    struct timespec ts = {0, 0};

    if (clock_gettime(CLOCK_REALTIME, &ts) != 0)
    {
        ROS_FATAL("Failed to poll realtime clock!");
    }

    ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
    ros::Duration period(1.0);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    realtime_tools::RealtimePublisher<std_msgs:Duration> publisher(barrett_nh, "loop_rate", 2);

    bool wam_ok = false;
    while(!g_quit && !wam_ok)
    {
        if (!barrett_robot.configure())
        {
            ROS_ERROR("Could not configure WAM!");
        }
        else if (!barrett_robot.start())
        {
            ROS_ERROR("Could not start WAM!");
        }
        else
        {
            ros::Duration(1.0).sleep();

            if (!barrett_robot.read(now, period))
            {
                ROS_ERROR("Could not read from WAM!");
            }
            else
            {
                wam_ok = true;
            }
        }

        ros::Duration(1.0).sleep();
    }

    // Construct the controller manager 
    ros::NodeHandle nh;
    controller_manager::ControllerManager manager(&barrett_robot, nh);

    uint32_t count = 0;

    //Run as fast as possible 
    while (!g_quit)
    {
        // Get the time / period 
        if (!clock_gettime(CLOCK_REALTIME, &ts))
        {
            now.sec = ts.tv_sec;
            now.nsec = ts.tv_nsec;
            period = now - last;
            last = now;
        }
        else 
        {
            ROS_FATAL("Failed to poll realtime clock!");
            break;
        }

        // Read the state from the WAM 
        if (!barrett_robot.read(now, period))
        {
            g_quit = true;
            break;
        }

        // update the controllers 
        manager.update(now, period);

        // Write the command to the WAM 
        barrett_robot.write(now, period);

        if (count++ > 1000)
        {
            if (publisher.trylock())
            {
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

