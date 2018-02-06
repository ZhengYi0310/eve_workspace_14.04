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
      calibrated_(false),
      ramp(NULL, 0.03), // Default Cartesian Velocity
      config_path_("") // Point to the default config path 
    {
        //TODO: Determine pre-existing calibration from ROS param server
    }

    bool BarrettHW::configure()
    {
        using namespace terse_roscpp;
        std::vector<std::string> product_names;

        // Get The URDF Model of Wam 
        std::string urdf_str;
        param::require(nh_, "robot_description_yi", urdf_str, "The URDF for the Barrett Wam arm.");
        urdf_model_.initString(urdf_str);

        // Load Parameters 
        param::require(nh_, "product_names", product_names, "The unique barrett product names.");

        for (std::vector<std::string>::const_iterator it = product_names.begin(); it != product_names.end(); it++)
        {
            const std::string &product_name = *it;
            ros::NodeHandle product_nh(nh_, "products/"+product_name);

            // Determine the bus for this product 
            std::string bus_name;
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
                bool config_path_found = param::get(nh_, "busses/" + bus_name +"/config", config_path_, "Path to a libbarrett configuration file.");

                // Create a new manager 
                barrett_manager.reset(new barrett::ProductManager(config_path_found ? config_path_.c_str() : NULL /* USE DEFAULT CONFIG OTHERWISE */));
            
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
                // Get the configuration for this type of arm. 
                ROS_INFO_STREAM("the conguration file path is " << config_path_ << " !");
                // Construct and store the wam interface
                barrett_manager->waitForWam(); //prompt on zeroing 
                barrett_manager->wakeAllPucks();

                const libconfig::Setting& wam_config = barrett_manager->getConfig().lookup(barrett_manager->getWamDefaultConfigPath());
                

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
                    ROS_ERROR_STREAM("Cound not find Wam on bus !");
                    continue;
                }
            }
            else if(product_type == "hand")
            {
                //bring up the biotac from here Maybe !!!!!!!!!!!!!!!!!**********************
                param::require(product_nh, "tactile", tactile_sensors_exist_, "Whether biotac sensors exist and need to be initialized.");

                //only bringup the biotacs when "tactile is set to true"
                if (tactile_sensors_exist_)
                {
                    //boost::shared_ptr<BarrettHW::BioTacDevices> biotac_devices(new BarrettHW::BioTacDevices());
                    biotac_devices_.reset(new BarrettHW::BioTacDevices());
                    biotac_devices_->interface.reset(new biotac::BioTacHandClass("left_hand_biotacs"));
                    biotac_devices_->interface->initBioTacSensors();

                    ROS_INFO("Collect the first batch and assign biotac values ...");
                    biotac_devices_->bt_hand_msg = biotac_devices_->interface->collectBatch();
                    biotac_devices_->assign_value();
                    ROS_INFO("Register biotac state handles for the biotac hardware interface.");
            
                     
                    for (int i = 0; i < (int)(biotac_devices_->bt_hand_msg).bt_data.size(); i++)
                    {
                        barrett_model::BiotacFingerStateHandle biotac_finger_state_handle((biotac_devices_->bt_serial_vec)[i],
                                                                                  &((biotac_devices_->bt_position_vec)[i]),
                                                                                  &((biotac_devices_->tdc_data_vec)[i]),
                                                                                  &((biotac_devices_->tac_data_vec)[i]),
                                                                                  &((biotac_devices_->pdc_data_vec)[i]),
                                                                                  &((biotac_devices_->pac_data_array)[i]),
                                                                                  &((biotac_devices_->electrode_data_array)[i]));
                        biotac_fingers_interface_.registerHandle(biotac_finger_state_handle);

                        ROS_INFO("Create and register handle for the >%i th< biotac sensor, with serial number >%s<, on cheetah position >%i<", (i + 1), (biotac_devices_->bt_serial_vec)[i].c_str(), (biotac_devices_->bt_position_vec)[i]);
                    }
                    
            
                }
                //ROS_ERROR_STREAM("Look ma, no hands!");
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

        /*
        // Register biotac state interface 
        if (tactile_sensors_exist_)
        {
            this->registerInterface(&biotac_fingers_interface_);
        }
        */

        // Set the configured flag 
        configured_ = true;

        return true;
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
        
        //Declare Barrett template units 
        BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
        
        // Give us pretty stack traces when things die
        barrett::installExceptionHandler();

        // Construct a new wam device (interface and state storage)
        boost::shared_ptr<BarrettHW::WamDevice<DOF> > wam_device(new BarrettHW::WamDevice<DOF>());

        //******************************
        // Get the Wam interface and the LowLevelWam interface
        std::vector<barrett::Puck*> wamPucks = barrett_manager->getWamPucks();
        wamPucks.resize(DOF); // Dicard all but the first DOF elements 

        // Construct a Wam or a LowLevelWamWrapper 
        wam_device->Wam.reset(new barrett::systems::LowLevelWamWrapper<DOF>(barrett_manager->getExecutionManager(), wamPucks, barrett_manager->getSafetyModule(), wam_config["low_level"]));
        wam_device->jpController.reset(new barrett::systems::PIDController<jp_type, jt_type>(wam_config["joint_position_control"]));
        wam_device->jvFilter.reset(new barrett::systems::FirstOrderFilter<jv_type>(wam_config["joint_velocity_filter"]));
        wam_device->kinematicsBase.reset(new barrett::systems::KinematicsBase<DOF>(wam_config["kinematics"]));
        wam_device->gravityCompensator.reset(new barrett::systems::GravityCompensator<DOF>(wam_config["gravity_compensation"]));
        wam_device->jtSum.reset(new barrett::systems::Summer<jt_type, 3>(true));
        wam_device->jvController1.reset(new barrett::systems::PIDController<jv_type, jt_type>(wam_config["joint_velocity_control"][0]));
        wam_device->ExposedOutput.reset(new barrett::systems::ExposedOutput<jt_type>(Eigen::Matrix<double, DOF, 1>::Zero()));

        // make connections between systems 
        barrett::systems::connect(wam_device->Wam->jpOutput, wam_device->jpController->feedbackInput);
        barrett::systems::connect(wam_device->Wam->jpOutput, wam_device->kinematicsBase->jpInput);
        barrett::systems::connect(wam_device->Wam->jvOutput, wam_device->jvFilter->input);
        barrett::systems::connect(wam_device->jvFilter->output, wam_device->kinematicsBase->jvInput);
        
        barrett::systems::connect(wam_device->jvFilter->output, wam_device->jvController1->feedbackInput); // connect this for now just in order to get joint velocities;

        //*************** Don't connect gravity compensator ouput yet, wait for later
        barrett::systems::connect(wam_device->kinematicsBase->kinOutput, wam_device->gravityCompensator->kinInput);
        //***************
        if (barrett_manager->getExecutionManager() != NULL)
        {
            // Keep the jvFilter updated so it will provide accurate values for
            // calls to getJointVelocities().
            barrett_manager->getExecutionManager()->startManaging(*(wam_device->jvFilter));
        }

        barrett::systems::connect(wam_device->ExposedOutput->output, wam_device->jtSum->getInput(JT_INPUT));
        barrett::systems::connect(wam_device->jpController->controlOutput, wam_device->jtSum->getInput(SC_INPUT));
        barrett::systems::connect(wam_device->jtSum->output, wam_device->Wam->input);

        // initially, tie inputs together for zero torque 
        barrett::systems::connect(wam_device->Wam->jpOutput, wam_device->jpController->referenceInput);

        // start the 500 hz loop
        barrett_manager->getExecutionManager()->start();

        // Wait for Shift-Activate 
        // Check rapidly in case the user wants to perform some action (like
		// enabling gravity compensation) immediately after Shift-activate.
        barrett_manager->getSafetyModule()->waitForMode(barrett::SafetyModule::ACTIVE);        

        //&(wam_device->interface) = &(wam_device->Wam->getLowLevelWam());
        //***********************
        
        wam_device->resolver_ranges = this->compute_resolver_ranges<DOF>(wam_device->Wam->getLowLevelWam());

        // Get URDF links starting at the product tip link 
        std::string tip_joint_name;
        param::require(product_nh, "tip_joint", tip_joint_name, "WAM tip joint in URDF.");
        boost::shared_ptr<const urdf::Joint> joint = urdf_model_.getJoint(tip_joint_name);

        // Resize joint names 
        wam_device->joint_names.resize(DOF);

        // Create joint handles starting at the tip 
        for (int i = DOF-1; i >=0; i--)
        {
            //ROS_INFO("joint name: %s, and the i is %lu", (joint->name).c_str(), i);
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

        barrett_manager->getExecutionManager()->startManaging(ramp); //starting ramp manager

        ROS_INFO("%zu-DOF WAM Left.", DOF);
        //jp_type jp_home = wam_device->Wam->getJointPositions();

        if (barrett_manager->foundHand()) //Does the following only if a BarrettHand is found 
        {
            boost::shared_ptr<BarrettHW::HandDevice> hand_device(new BarrettHW::HandDevice());       
            hand_device->interface = barrett_manager->getHand();
            // Move j3 in order to give room for hand initialization 
            //jp_type jp_init = wam_device->Wam->getJointPositions();
            //jp_init[3] -= 0.35;
            usleep(500000);
            //wam_device->Wam->moveTo(jp_init);

            usleep(500000);
            hand_device->interface->initialize();
            hand_device->interface->update();

            //ROS_INFO("Closing the BarrenttHand Grasp");
            hand_device->interface->close(barrett::Hand::GRASP, false);

            wam_device->hand_device = hand_device;
        }

        //Compensate the gravity here
        barrett::systems::forceConnect(wam_device->gravityCompensator->output, wam_device->jtSum->getInput(GRAVITY_INPUT));        
  
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
        
        // Reset all biotac sensor states 
        if (tactile_sensors_exist_)
        {
            biotac_devices_->reset();
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

    bool BarrettHW::read(const ros::Time time, const ros::Duration period)
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

        // Collect biotac batch data from each cheetah 
        // TODO what if there are more than one cheetah device ?
        //biotac_devices_->bt_hand_msg = biotac_devices_->interface->collectBatch();
        //biotac_devices_->assign_value();
        
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
        BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);    
        /*
        // Poll the hardware
        try 
        {
            //(&(device->interface))->update();
            (device->Wam->getLowLevelWam()).update();
        }
        catch (const std::runtime_error &e)
        {
            if ((device->Wam->getLowLevelWam()).getSafetyModule() != NULL && (device->Wam->getLowLevelWam()).getSafetyModule()->getMode(true) == barrett::SafetyModule::ESTOP)
            {
                ROS_ERROR_STREAM("systems::LowLevelWamWrapper::Source::operate(): E-stop! Cannot communicate with Pucks!");
                return false;
            }
            else 
            {
                throw;
            }
        }
        */

        // Get raw state 
        //Eigen::Matrix<double, DOF, 1> 
        const jp_type raw_positions = (device->Wam->getLowLevelWam()).getJointPositions();
        //Eigen::Matrix<double, DOF, 1> 
        jv_type raw_velocities;

        {
            BARRETT_SCOPED_LOCK(device->Wam->getEmMutex());
            {
                // Return filtered velocity, if avaiable.
                if ((device->jvController1->feedbackInput).valueDefined())
                {
                    raw_velocities = (device->jvController1->feedbackInput).getValue();
                }
                else 
                {
                    raw_velocities = (device->Wam->getLowLevelWam()).getJointVelocities();
                }
            }
        }
        
        // Smooth velocity 
        // TODO: parameterrize time constant
        for (size_t i = 0; i < DOF; i++)
        {
            device->joint_velocities(i) = filters::exponentialSmoothing(raw_velocities(i), device->joint_velocities(i), 0.8);
        }
        
        // Store position 
        for (size_t i = 0; i < DOF; i++)
        {
            device->joint_positions(i) = raw_positions(i);
        }
       
       
        /*
        // Read resolver angles 
        std::vector<barrett::Puck*> pucks = (device->Wam->getLowLevelWam()).getPucks();
        for (size_t i = 0; i < pucks.size(); i++)
        {
            device->resolver_angles(i) = pucks[i]->getProperty(barrett::Puck::MECH);
        }
        */
         
        return true;
    }

    template <size_t DOF>
    void BarrettHW::write_wam(const ros::Time time, const ros::Duration period, boost::shared_ptr<BarrettHW::WamDevice<DOF> > device)
    {
        BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
        
        //first disconnect the Exposed system and jtSum 
        barrett::systems::disconnect(device->jtSum->getInput(JT_INPUT));
        

        for (size_t i = 0; i < DOF; i++)
        {
            if (std::abs(device->joint_effort_cmds(i)) > device->effort_limits(i))
            {
                ROS_WARN_STREAM("Commanded torque (" << device->joint_effort_cmds(i) << ") of joint (" << i << ") exceeds safety limits! They have been truncated to: +/- " << device->effort_limits(i));
            
            // Truncate this joint torque 
            device->joint_effort_cmds(i) = std::max(std::min(device->joint_effort_cmds(i), device->effort_limits(i)), -1.0*device->effort_limits(i));
            }
        }

        // Set the value of the ExposedOutput and reconnect them 
        barrett::systems::connect(device->ExposedOutput->output, device->jtSum->getInput(JT_INPUT)); 

        /*****************************
        // Don't use setTorques in another realtime thread ever!!!!!!!!!!!
        device->Wam->getLowLevelWam().setTorques(device->joint_effort_cmds);
        ******************************/
    }

    bool BarrettHW::wait_for_mode(barrett::SafetyModule::SafetyMode mode, ros::Duration timeout, ros::Duration poll_duration)
    {
        ros::Time polling_start_time = ros::Time::now();
        ros::Rate poll_rate(1.0 / poll_duration.toSec());

        for (ManagerMap::iterator it = barrett_managers_.begin(); it != barrett_managers_.end(); it++)
        {
            while (ros::ok() && (ros::Time::now() - polling_start_time < timeout) && (it->second->getSafetyModule()->getMode() != mode))
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
    //RT_TASK task;
    //rt_task_shadow(&task, "GroupWAM", 49, 0); // lower than the 500hz loop 
    barrett::PeriodicLoopTimer loopTimer(0.004, 49);

    // initialize ROS 
    ros::init(argc, argv, "wam_server", ros::init_options::NoSigintHandler);

    // Add custom signal handlers 
    signal(SIGTERM, quitRequested);
    signal(SIGINT, quitRequested);
    signal(SIGHUP, quitRequested);

    // Construct the wam structure 
    ros::NodeHandle barrett_nh("barrett");
    barrett_hw::BarrettHW barrett_robot(barrett_nh);
    //barrett_robot.configure();
    //barrett_robot.start();

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

    realtime_tools::RealtimePublisher<std_msgs::Duration> publisher(barrett_nh, "loop_rate", 2);

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
    //rt_task_set_periodic(NULL, TM_NOW, static_cast<RTIME>(0.002 * 1e9));
    while (!g_quit)
    {
        // Explicit interruption point 
        boost::this_thread::interruption_point();
        loopTimer.wait();

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
        

        std::cout << count << std::endl;
        
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

