#ifndef __WAM_DMP_CONTROLLER_KINEMATIC_CHAIN_CONTROLLER_BASE_H
#define __WAM_DMP_CONTROLLER_KINEMATIC_CHAIN_CONTROLLER_BASE_H

#include <control_msgs/JointControllerState.h> // TODO: state message for all controllers?

#include <urdf/model.h>
//#include <barrett_hw/wam_server.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <vector>

namespace wam_dmp_controller
{   
	template<typename JI>
	class KinematicChainControllerBase : public controller_interface::Controller<JI>
	{
		public:
			KinematicChainControllerBase() {}
			~KinematicChainControllerBase() {}

			bool init(JI *hw, ros::NodeHandle &n);

			protected:
				ros::NodeHandle nh_;
				KDL::Chain kdl_chain_;
        		KDL::Vector gravity_; 
        		KDL::JntArrayAcc joint_msr_states_, joint_des_states_;  // joint states (measured and desired)

        		struct limits_
				{
					KDL::JntArray min;
					KDL::JntArray max;
					KDL::JntArray center;
				} joint_limits_;

				std::vector<typename JI::ResourceHandleType> joint_handles_;
                        std::vector<typename JI::ResourceHandleType> joint_acceleration_handles_; // For acceleration reading
        		std::vector<typename JI::ResourceHandleType> joint_stiffness_handles_;
        		std::vector<typename JI::ResourceHandleType> joint_damping_handles_;
        		std::vector<typename JI::ResourceHandleType> joint_set_point_handles_;
        
        		bool getHandles(JI *hw);
	};

	template <typename JI>
    bool KinematicChainControllerBase<JI>::init(JI *hw, ros::NodeHandle &n)
    {
    	nh_ = n;

        // get URDF and name of root and tip from the parameter server
        std::string robot_description, root_seg_name, tip_seg_name;
        std::string urdf_string;
    
        if (!ros::param::search(nh_.getNamespace(), "robot_description_yi", robot_description))
        {
            ROS_ERROR_STREAM("KinematicChainControllerBase: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description_yi)");
            return false;
        }

        if (!nh_.getParam("root_link", root_seg_name))
        {
            ROS_ERROR_STREAM("KinematicChainControllerBase: No root name found on parameter server ("<<n.getNamespace()<<"/root_link)");
            return false;
        }

        if (!nh_.getParam("tip_link", tip_seg_name))
        {
            ROS_ERROR_STREAM("KinematicChainControllerBase: No tip name found on parameter server ("<<n.getNamespace()<<"/tip_link)");
            return false;
        }

        // Get the gravity vector (direction and magnitude)
        gravity_ = KDL::Vector::Zero();
        gravity_(2) = -9.81;
        
        if (nh_.hasParam(robot_description))
        {
            nh_.getParam(robot_description.c_str(), urdf_string);
        }
        else
        {
            ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
            n.shutdown();
            return false;
        }

        if (urdf_string.size() == 0)
        {
            ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
            n.shutdown();
            return false;
        }
        urdf::Model urdf_model_;
        //if (!urdf.initParamWithinNodeHandle("robot_description", n))
        if (!urdf_model_.initString(urdf_string))
        {
            ROS_ERROR("Failed to parse urdf file");
            n.shutdown();
            return false;
        }
        ROS_INFO("Successfully parsed urdf file");

        KDL::Tree kdl_tree_;
        if (!kdl_parser::treeFromUrdfModel(urdf_model_, kdl_tree_)) 
        {
            ROS_ERROR("Could not convert the urdf model into a KDL::Tree!");
            n.shutdown();
            return false;
        }

        if (!kdl_tree_.getChain(root_seg_name, tip_seg_name, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  "<<root_seg_name<<" --> "<<tip_seg_name);
            ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
            ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for( it=segment_map.begin(); it != segment_map.end(); it++ )
              ROS_ERROR_STREAM( "    "<<(*it).first);

            return false;
        }

        ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
        ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
        
        // Parsing joint limits from urdf model along kdl chain
        boost::shared_ptr<const urdf::Link> link_ = urdf_model_.getLink(tip_seg_name);
        boost::shared_ptr<const urdf::Joint> joint_;
        joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
        joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
        joint_limits_.center.resize(kdl_chain_.getNrOfJoints());

        int index;
        
    	for (int i = 0; i < kdl_chain_.getNrOfJoints() && link_; i++)
    	{
        	joint_ = urdf_model_.getJoint(link_->parent_joint->name);  
        	ROS_DEBUG("Getting limits for joint: %s", joint_->name.c_str());
        	index = kdl_chain_.getNrOfJoints() - i - 1;

        	joint_limits_.min(index) = joint_->limits->lower;
        	joint_limits_.max(index) = joint_->limits->upper;
        	joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index))/2;

        	link_ = urdf_model_.getLink(link_->getParent()->name);
    	}

    	

	    std::string controller_type;
 	    nh_.getParam("type", controller_type);
	    ROS_INFO("Controller %s succssfully loaded", controller_type.c_str());

    	getHandles(hw);        
    	ROS_DEBUG("Number of joints in handle = %lu", joint_handles_.size() );
        for (uint32_t i = 0; i < joint_handles_.size(); i++)
        {
            ROS_INFO("The name of for the handle %lu is: %s", i + 1, joint_handles_[i].getName().c_str());
        }

        // Get joint handles for all of the joints in the chain
        
    	joint_msr_states_.resize(kdl_chain_.getNrOfJoints());
    	joint_des_states_.resize(kdl_chain_.getNrOfJoints());

    	return true;
    }    

    template <typename JI>
    bool KinematicChainControllerBase<JI>::getHandles(JI *hw)
    {
        for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
        {
            if ( it->getJoint().getType() != KDL::Joint::None )
            {
                joint_handles_.push_back(hw->getHandle(it->getJoint().getName()));
                ROS_DEBUG("%s", it->getJoint().getName().c_str() );
            }
        }        
        return true;
    }
}
#endif
