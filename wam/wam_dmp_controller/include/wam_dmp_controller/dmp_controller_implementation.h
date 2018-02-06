/*************************************************************************
	> File Name: dmp_controller_implementation.h
	> Author:Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 06 Jun 2017 10:13:27 AM PDT
 ************************************************************************/

#ifndef _DMP_CONTROLLER_IMPLEMENTATION_H
#define _DMP_CONTROLLER_IMPLEMENTATION_H

// system includes 
#include <string>
#include <vector>

// ros includes 
#include <ros/ros.h>
#include <rosrt/rosrt.h>

#include <geometry_msgs/PoseStamped.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/constants.h>
#include <usc_utilities/param_server.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>

// local includes 
#include <wam_dmp_controller/dmp_controller.h>
#include <wam_dmp_controller/variable_name_map.h>
#include "wam_dmp_controller/filtered_subscriber.h"

namespace wam_dmp_controller
{
    template<class DMPType>
    class DMPControllerImplementation : public DMPController 
    {
        public:
            
            DMPControllerImplementation() {};
            virtual ~DMPControllerImplementation() {};

            /*!
             * @param controller_name 
             * @param variable_names
             * return True on success, otherwise False 
             */
            bool initialize(const std::string& controller_name,
                            const std::vector<std::string>& variable_names);

            /*!
             * @param new_start
             * @return True on success, otherwise False 
             * REAL-TIME REQUIREMENTS
             */
            bool changeDMPStart(const Eigen::VectorXd& new_start);

            /*!
             * @return True on success, otherwise False 
             * REAL-TIME REQUIREMENTS
             */
            bool newDMPReady();

            /*!
             * @return True on success, otherwise False 
             * REAL-TIME REQUIREMENTS
             */
            bool isRunning(Eigen::VectorXd& desired_positions,
                           Eigen::VectorXd& desired_velocities,
                           Eigen::VectorXd& desired_accelerations);

            /*!
             * @return 
             * REAL-TIME REQUIREMENTS
             */
            bool getDMP(dmp_lib::DMPBasePtr& dmp);

            /*!
             * @return 
             * REAL-TIME REQUIREMENTS
             */
            bool stop();

        private:
            
            /*!
             * REAL-TIME REQUIREMENTS
             * @return 
             */
            typename DMPType::DMPPtr getDMP();

            /*!Callback function to filter incoming messages 
             * @param msg 
             * @param dmp 
             * return 
             */
            bool filter(const typename DMPType::DMPMsgConstPtr& msg,
                        const typename DMPType::DMPPtr dmp);

            /*!
             * @param status 
             * @param movement_finished
             * @param end_time 
             * REAL-TIME REQUIREMENTS
             */
            void publishStatus(const int status, const bool movement_finished, const ros::Time& end_time);

            /*!
             * @param desired_positions
             * @param desired_velocities
             * @param desired_accelerations
             * return True on success, otherwise False 
             * REAL-TIME REQUIREMENTS
             */
            bool setVariables(Eigen::VectorXd& desired_positions,
                              Eigen::VectorXd& desired_velocities,
                              Eigen::VectorXd& desired_accelerations);

            /*!
             * @param dmp 
             * @param strict 
             * return True on success, otherwise False 
             * REAL-TIME REQUIREMENTS
             */
            bool setDMP(typename DMPType::DMPPtr dmp, bool strict = true);

            /*!
             * @return True on success, otherwise False
             * REAL-TIME REQUIREMENTS
             */
            bool changeGoal();

            /*!
             */
            typename DMPType::DMPPtr dmp_;

            /*!
             */
            std::string abs_bag_file_name_;

            bool movement_finished_;

            /*!
             */
            //rosrt::Subscriber<geometry_msgs::PoseStamped> dmp_goal_subscriber_; 
            //rosrt::FilteredSubscriber<typename DMPType::DMPMsg, typename DMPType::DMP> dmp_filtered_subscriber_;
    };

    template<class DMPType>
    bool DMPControllerImplementation<DMPType>::filter(const typename DMPType::DMPMsgConstPtr& msg,
                                                      const typename DMPType::DMPPtr dmp)
    {
        return DMPType::initFromMessage(dmp, *msg);
    }

    template<class DMPType>
    bool DMPControllerImplementation<DMPType>::initialize(const std::string& controller_name,
                                                          const std::vector<std::string>& variable_names)
    {
        // error checking 
        if (variable_names.empty())
        {
            ROS_ERROR("%s: Cannot initialize a DMP controller with empty variable names!", controller_name.c_str());
            return false;
        }
        ROS_DEBUG("Initializing a DMP controller >%s< with the following variable names:", controller_name.c_str());
        for (int i = 0; i < (int)variable_names.size(); i++)
        {
            ROS_DEBUG(">%s<", variable_names[i].c_str());
        }

        ros::NodeHandle controller_node_handle(controller_name);
        std::vector<std::string> controller_variable_names;
        ROS_VERIFY(usc_utilities::read(controller_node_handle, "trajectory/variable_names", controller_variable_names));
        ROS_VERIFY(usc_utilities::read(controller_node_handle, "abs_dmp_bagfile_location", abs_bag_file_name_));
        ROS_VERIFY(variable_name_map_.initialize(variable_names, controller_variable_names));

        entire_desired_positions_ = Eigen::VectorXd::Zero(variable_names.size());
        entire_desired_velocities_ = Eigen::VectorXd::Zero(variable_names.size());
        entire_desired_accelerations_ = Eigen::VectorXd::Zero(variable_names.size());

        ros::NodeHandle node_handle;
        //rosrt::init(); // before using any part of rosrt, this one must be called!!!!!
        //ROS_INFO("Initialize the realtime subscriber...");
        //ROS_VERIFY(dmp_filtered_subscriber_.initialize(100, node_handle, controller_name + "/command", boost::bind(&DMPControllerImplementation<DMPType>::filter, this, _1, _2)));
        //ROS_VERIFY(dmp_goal_subscriber_.initialize(100, node_handle, controller_name + "/goal"));
        //ROS_INFO("realtime subscriber initialization done!");

        // Start the real time publisher 
        dmp_status_publisher_.reset(new realtime_tools::RealtimePublisher<dynamic_movement_primitive::ControllerStatusMsg>(node_handle, controller_name + "/status", 10));
        
        dmp_is_being_executed_ = false;
        dmp_is_set_ = false;
        movement_finished_ = false;

        return (initialized_ = true);
    }

    //REAL-TIME REQUIREMENTS
    template<class DMPType>
    void DMPControllerImplementation<DMPType>::publishStatus(const int status, const bool movement_finished, const ros::Time& end_time)
    {
        if(dmp_status_publisher_ && dmp_status_publisher_->trylock())
        {
            dmp_status_publisher_->msg_.status = status;
            dmp_status_publisher_->msg_.id = dmp_->getId(); 
            dmp_status_publisher_->msg_.percent_complete = dmp_->getProgress(); 
            dmp_status_publisher_->msg_.start_time = start_time_;
            if (movement_finished)
            {
                dmp_status_publisher_->msg_.end_time = end_time;
            }
            dmp_status_publisher_->unlockAndPublish();
        }
    }

    //REAL-TIME REQUIREMENTS
    template<class DMPType>
    bool DMPControllerImplementation<DMPType>::setVariables(Eigen::VectorXd& desired_positions,
                                                            Eigen::VectorXd& desired_velocities,
                                                            Eigen::VectorXd& desired_accelerations)
    {
        for (int i = 0; i < num_variables_used_; i++)
        {
            int index = 0;
            if (!variable_name_map_.getSupportedVariableIndex(i, index))
            {
                ROS_ERROR("No corresponding index!");
                return false;
            }

            desired_positions(index) = entire_desired_positions_(i); //desired_positions(i) = entire_desired_positions_(index)
            desired_velocities(index) = entire_desired_velocities_(i);
            desired_accelerations(index) = entire_desired_accelerations_(i);
        }
        return true;
    }
    
    // REAL-TIME REQUIREMENTS
	
    template<class DMPType>
    typename DMPType::DMPPtr DMPControllerImplementation<DMPType>::getDMP()
    {
        ROS_ASSERT(initialized_);
        return dmp_;
    }
    
    //REAL-TIME REQUIREMENTS
    template<class DMPType>
    bool DMPControllerImplementation<DMPType>::getDMP(dmp_lib::DMPBasePtr& dmp)
    {
        ROS_ASSERT(initialized_);
        dmp = static_cast<dmp_lib::DMPBasePtr>(dmp_); 
        return true;
    }

    //REAL-TIME REQUIREMENTS
    template<class DMPType>
    bool DMPControllerImplementation<DMPType>::setDMP(typename DMPType::DMPPtr dmp, bool strict)
    {
        if (dmp->setup())
        {
            variable_name_map_.reset();
            num_variables_used_ = 0;
            for (int i = 0; i < dmp->getNumTransformationSystems(); ++i)
            {
                for (int j = 0 ; j < dmp->getTransformationSystem(i)->getNumDimensions(); j++)
                {
                    if (!variable_name_map_.set(dmp->getTransformationSystem(i)->getName(j), num_variables_used_))
                    {
                        if (strict)
                        {
                            ROS_ERROR("Received DMP variable name >%s< is not handled by this DMP controller (real-time violation).", dmp->getTransformationSystem(i)->getName(j).c_str());
                            return false;
                        }
                    }
                    else
                    {
                        num_variables_used_++;
                    }
                }
            }
            start_time_ = ros::Time::now();
            dmp_.reset(); 
            dmp_ = dmp; 
            dmp_is_set_ = true;
            return (dmp_is_being_executed_ = true);
        }
        else
        {
            ROS_ERROR("DMP is not set up (real-time violation)!");
        }

        return true;
    }

    // REAL-TIME REQUIREMENTS
    template<class DMPType>
    bool DMPControllerImplementation<DMPType>::newDMPReady()
    {
        // Maybe change it to read the DMP from binary file
        if (!dmp_is_being_executed_ && !movement_finished_)
        {
            typename DMPType::DMPPtr dmp;
            //dmp = dmp_filtered_subscriber_.poll();  Can't use the poll based subscriber from rosrt because it messes up with the threading 
            DMPType::readFromDisc(dmp, abs_bag_file_name_);
            if (dmp)
            {
                return setDMP(dmp);
            }
        }
        return false;
    }

    // REAL-TIME REQUIREMENTS
    template<class DMPType>
    bool DMPControllerImplementation<DMPType>::isRunning(Eigen::VectorXd& desired_positions,
                                                Eigen::VectorXd& desired_velocities,
                                                Eigen::VectorXd& desired_accelerations)
    {
        if (dmp_is_set_)
        {
            ROS_VERIFY(changeGoal());
            bool movement_finished = false;
			
            if (!dmp_->propagateStep(entire_desired_positions_, entire_desired_velocities_, entire_desired_accelerations_, movement_finished))
            {
                // something went wrong 
                publishStatus(dynamic_movement_primitive::ControllerStatusMsg::FAILED, false, ros::Time::now());
                dmp_is_being_executed_ = false;
            } 

            if (!setVariables(desired_positions, desired_velocities, desired_accelerations))
            {
                // something went wrong
                publishStatus(dynamic_movement_primitive::ControllerStatusMsg::FAILED, false, ros::Time::now());
                dmp_is_being_executed_ = false;                
            }

            if (dmp_is_being_executed_ && movement_finished)
            {
                publishStatus(dynamic_movement_primitive::ControllerStatusMsg::FINISHED, movement_finished, ros::Time::now());
                dmp_is_being_executed_ = false;
                movement_finished_ = true;
            }

            return dmp_is_being_executed_;
        }
        else 
        {
            ROS_ERROR("DMP is not set (real-time violation)!");
            return false;
        }
    }

    // REAL-TIME REQUIREMENTS
    template<class DMPType>
    bool DMPControllerImplementation<DMPType>::changeGoal()
    {
        geometry_msgs::PoseStamped::ConstPtr goal_pose; //= dmp_goal_subscriber_.poll();
        if (goal_pose)
        {
            for (int i = 0; i < num_variables_used_; i++)
            {
                int index = 0;
                if (!variable_name_map_.getSupportedVariableIndex(i, index))
                {
                    ROS_ERROR("Could not get variable mapping. This should never happen (real-time violation).");
                    return false;
                }
                index++;
                
                int local_index = index - 1;
                if ((local_index >= 0) && (local_index < usc_utilities::Constants::N_CART))
                {
                    if (local_index == usc_utilities::Constants::X)
                    {
                        ROS_VERIFY(dmp_->changeGoal(goal_pose->pose.position.x, local_index));
                    }
                    else if (local_index == usc_utilities::Constants::Y)
                    {
                        ROS_VERIFY(dmp_->changeGoal(goal_pose->pose.position.y, local_index));
                    }
                    else if (local_index == usc_utilities::Constants::Z)
                    {
                        ROS_VERIFY(dmp_->changeGoal(goal_pose->pose.position.z, local_index));
                    }
                }

                else if ((local_index >= usc_utilities::Constants::N_CART) && (local_index < usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT))
                {
                    if (local_index == usc_utilities::Constants::N_CART + usc_utilities::Constants::QW)
                    {
                        ROS_VERIFY(dmp_->changeGoal(goal_pose->pose.orientation.w, local_index));
                    }
                    else if (local_index == usc_utilities::Constants::N_CART + usc_utilities::Constants::QX)
                    {
                        ROS_VERIFY(dmp_->changeGoal(goal_pose->pose.orientation.w, local_index));
                    }
                    else if (local_index == usc_utilities::Constants::N_CART + usc_utilities::Constants::QY)
                    {
                        ROS_VERIFY(dmp_->changeGoal(goal_pose->pose.orientation.y, local_index));
                    }
                    else if (local_index == usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ)
                    {
                        ROS_VERIFY(dmp_->changeGoal(goal_pose->pose.orientation.z, local_index));
                    }
                }
				
            }
        }

        return true;
    }

    template<class DMPType>
    bool DMPControllerImplementation<DMPType>::changeDMPStart(const Eigen::VectorXd& new_start)
    {
        return getDMP()->changeStart(new_start);
    }

    template<class DMPType>
    bool DMPControllerImplementation<DMPType>::stop()
    {
        dmp_is_being_executed_ = false;
        dmp_is_set_ = false;
        return true;
    }
}
#endif
