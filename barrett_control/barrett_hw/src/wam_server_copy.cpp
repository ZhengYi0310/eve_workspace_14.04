/*************************************************************************
	> File Name: wam_server_copy.cpp
	> Author: Yi Zheng
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 01 Jun 2017 01:44:02 PM PDT
 ************************************************************************/
#include <string>
#include <limits>
#include <cmath>
#include <cassert>
#include <iostream>

#include <barrett_hw/wam_server.h>

namespace barrett_hw
{
    BarrettHW::BarrettHW(ros::NodeHandle nh) :
      nh_(nh),
      configured_(false),
      calibrated_(false),
      ramp(NULL, 0.03), // Default Cartesian Velocity
      config_path_(""), // Point to the default config path 
      joint_states_to_biotac_counter_(0)
    {};

    bool BarrettHW::assignKDLTree(KDL::Tree kdl_tree, size_t num_of_segments)
    {
        kdl_tree_ = kdl_tree;
        if (kdl_tree_.getNrOfSegments() == num_of_segments)
        {
            return true;
        }
        else 
        {
            ROS_ERROR("error when assinging kdl tree!");
            return false;
        }
    }

    void BarrettHW::printLink(const KDL::SegmentMap::const_iterator& link, const std::string& prefix)
    {
        cout << prefix << "- Segment " << GetTreeElementSegment(link->second).getName() << " has " << GetTreeElementChildren(link->second).size() << " children" << endl;
        for (unsigned int i=0; i < GetTreeElementChildren(link->second).size(); i++)
        {
            printLink(GetTreeElementChildren(link->second)[i], prefix + "  ");
        }
    }

    bool BarrettHW::configure()
    {
        using namespace terse_roscpp;
        std::vector<std::string> product_names;

        // Get The URDF Model of Wam 
        param::require(nh_, "robot_description_yi", urdf_str_, "The URDF for the Barrett Wam arm.");
        param::require(nh_, "urdf_file_path", urdf_path_, "The path to the Barrett Wam URDF file.");
        
        if (!urdf_model_.initString(urdf_str_))
        {
            ROS_ERROR("Could note generate the urdf model for the robot");
        }
        
        // Load Product Related Parameters 
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
                param::require(product_nh, "tactile", tactile_sensors_exist_, "Whether biotac sensors exist and need to be initialized.");
                
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
        this->registerInterface(&arm_pose_state_interface_);
        // Register the hardware interface for the robot publisher 
        this->registerInterface(&robot_state_interface_);
        // Register biotac state interface 
        if (tactile_sensors_exist_)
        {
            this->registerInterface(&biotac_hand_interface_);
        }
        

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
        // feed KDL::Segment name !!! not joint name!!! careful!!!!
        param::require(product_nh, "tip_segment", wam_device->tip_seg_name, "WAM tip link in URDF.");
        param::require(product_nh, "root_segment", wam_device->root_seg_name, "WAM root link in URDF.");
        param::require(product_nh, "tip_joint", wam_device->tip_joint_name, "WAM tip joint in URDF.");

        if (!kdl_parser::treeFromUrdfModel(urdf_model_, kdl_tree_)) 
        {
            ROS_ERROR("Could not convert the urdf model into a KDL::Tree!");
        }
        printLink(kdl_tree_.getRootSegment(), "");

        if (!kdl_tree_.getChain(wam_device->root_seg_name, wam_device->tip_seg_name, wam_device->kdl_chain_))
        {
            ROS_ERROR("Could not extract chain between %s and %s from kdl tree", wam_device->root_seg_name.c_str(), wam_device->tip_seg_name.c_str());
        }
        
        if (static_cast<size_t>(wam_device->kdl_chain_.getNrOfJoints()) != DOF)
        {
            ROS_ERROR("For now, the KDL chain needs to have >%lu< arm joints, but only has >%lu<", DOF, static_cast<size_t>(wam_device->kdl_chain_.getNrOfJoints()));
        }
        else 
        {
            ROS_INFO_STREAM("The KDL Chain for this device has " << static_cast<int>(wam_device->kdl_chain_.getNrOfJoints()) << " joints!");
        }
        
        // Initialize all the KDL solvers and matrix         
        wam_device->jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(wam_device->kdl_chain_));
        wam_device->jnt_to_twist_solver_.reset(new KDL::ChainFkSolverVel_recursive(wam_device->kdl_chain_));
        wam_device->jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(wam_device->kdl_chain_));
        wam_device->kdl_chain_jacobian_.resize(DOF);
        wam_device->kdl_current_joint_positions_.resize(DOF);
        wam_device->kdl_current_joint_velocities_.resize(DOF);

        // Register the arm pose state interface 
        std::string product_type;
        param::require(product_nh, "type", product_type, "Barrett product type [wam, bhand].");        
        wam_device->kdl_pose_measured_ = KDL::Frame::Identity();
        wam_device->kdl_twist_measured_ = KDL::Twist::Zero();
        barrett_model::ArmPoseStatesHandle arm_pose_states_handle(product_type, &(wam_device->root_seg_name), &(wam_device->kdl_pose_measured_), &(wam_device->kdl_twist_measured_));
        arm_pose_state_interface_.registerHandle(arm_pose_states_handle);
        
                
        boost::shared_ptr<const urdf::Joint> joint = urdf_model_.getJoint(wam_device->tip_joint_name);

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
                    ROS_ERROR_STREAM("Run out of joints while parsing URDF starting at joint: " << wam_device->tip_joint_name);
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
        barrett_model::RobotStateHandle robot_state_handle(product_nh.getNamespace(), &state_interface_, &arm_pose_state_interface_);
        robot_state_interface_.registerHandle(robot_state_handle);

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

            ROS_INFO("Closing the BarrenttHand Grasp");
            usleep(500000);
            hand_device->interface->close(barrett::Hand::GRASP, false);
            //usleep(500000);
            //hand_device->interface->close(barrett::Hand::F1, false);
            //usleep(500000);
            //hand_device->interface->close(barrett::Hand::F3, false);

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
        
        //TODO seems like not a good idea here, update() and collect batch together will mess up the rt_performance
        // Poll the hardware
        /*
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
        jt_type raw_torques;
        //Eigen::Matrix<double, DOF, 1> 
        jv_type raw_velocities;
        
        {
            BARRETT_SCOPED_LOCK(device->Wam->getEmMutex());
            {
                if ((device->Wam->input).valueDefined())
                {
                    raw_torques = (device->Wam->input).getValue();
                }
            }
        }
        
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
            device->kdl_current_joint_velocities_.qdot(i) = device->joint_velocities(i);
            device->joint_effort_cmds(i) = raw_torques(i);
        }
        
        // Store position 
        for (size_t i = 0; i < DOF; i++)
        {
            device->joint_positions(i) = raw_positions(i);
            device->kdl_current_joint_velocities_.q(i) = raw_positions(i);
            device->kdl_current_joint_positions_(i) = raw_positions(i);
        }

        // Get the jacobian 
        device->jnt_to_jac_solver_->JntToJac(device->kdl_current_joint_positions_, device->kdl_chain_jacobian_);

        // Get the cartesian pose 
        device->jnt_to_pose_solver_->JntToCart(device->kdl_current_joint_positions_, device->kdl_pose_measured_);

        // Get the cartesian velocti
        device->jnt_to_twist_solver_->JntToCart(device->kdl_current_joint_velocities_, device->frame_vel_measured_);
        device->kdl_twist_measured_ = device->frame_vel_measured_.deriv();
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
            //device->joint_effort_cmds(i) = std::max(std::min(device->joint_effort_cmds(i), device->effort_limits(i)), -1.0*device->effort_limits(i));
            }
        }

        // Set the value of the ExposedOutput and reconnect them 
        //device->ExposedOutput->setValue(device->joint_effort_cmds);
        //barrett::systems::connect(device->ExposedOutput->output, device->jtSum->getInput(JT_INPUT)); 

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
    
    // initialize ROS 
    ros::init(argc, argv, "wam_server", ros::init_options::NoSigintHandler);
    
    // Add custom signal handlers 
    signal(SIGTERM, quitRequested);
    signal(SIGINT, quitRequested);
    signal(SIGHUP, quitRequested);
    
    // Construct the wam structure 
    ros::NodeHandle barrett_nh("barrett");
    barrett_hw::BarrettHW barrett_robot(barrett_nh);
          
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
    
    bool verbose;
    double period_rt_loop;
    barrett_nh.param("verbose", verbose, false);
    barrett_nh.param("period", period_rt_loop, 0.004); //Default at 250hz
    // Don't put this before ProductManager.startExecutionManager()    
    barrett::PeriodicLoopTimer loopTimer(period_rt_loop, 49); // Set the priority lower than the 500hz thread
    uint32_t period_us = period_rt_loop * 1e6;
    double start;
    uint32_t duration;
    uint32_t min = std::numeric_limits<size_t>::max();
    uint32_t max = 0;
    uint64_t sum = 0;
    uint64_t sumSq = 0;
    uint32_t loopCount = 0;
    uint32_t overrruns = 0;
    uint32_t missedReleasePoints = 0;
    
    // Construct the controller manager 
    ros::NodeHandle nh;
    controller_manager::ControllerManager manager(&barrett_robot, nh);

    //Run as fast as possible
    //rt_task_set_periodic(NULL, TM_NOW, static_cast<RTIME>(0.002 * 1e9));
    try
    {
        uint32_t count = 0;        
        while (!g_quit)
        {
            boost::this_thread::interruption_point();
            missedReleasePoints += loopTimer.wait();
            start = barrett::highResolutionSystemTime();
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

            duration = (barrett::highResolutionSystemTime() - start) * 1e6;
            if (duration < min)
            {
                min = duration;
            }
            if (duration > max)
            {
                max = duration;
            }
            sum += duration;
            sumSq += duration * duration;
            ++loopCount;
            if (duration > period_us)
            {
                overrruns++;
            }
        
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
    }
    catch (const boost::thread_interrupted& e)
    {}

    publisher.stop();

    if (verbose)
    {
        double mean = (double)sum / loopCount;
        double stdev = std::sqrt(((double)sumSq / loopCount) - mean * mean);

        ROS_INFO("RealTimeExecutionManager control-loop stats (microseconds):");
        ROS_INFO_STREAM("  target period " << period_us);
        ROS_INFO_STREAM("  min = " << min);
        ROS_INFO_STREAM("  max = " << max);
        ROS_INFO_STREAM("  ave = " << mean);
        ROS_INFO_STREAM("  stdev = " << stdev);
        ROS_INFO_STREAM(" num total cycles = " << loopCount);
        ROS_INFO_STREAM(" num missed release points = " << missedReleasePoints);
        ROS_INFO_STREAM(" num overrruns = " << overrruns);
    }
    std::cerr<<"Stpping spinner..."<<std::endl;
    spinner.stop();

    std::cerr<<"Stopping WAM..."<<std::endl;
    barrett_robot.stop();

    std::cerr<<"Cleaning up WAM..."<<std::endl;
    barrett_robot.cleanup();

    std::cerr<<"Poka!"<<std::endl;
    

    return 0;
}

