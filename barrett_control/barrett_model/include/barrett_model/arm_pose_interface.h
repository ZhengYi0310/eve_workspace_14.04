/*************************************************************************
	> File Name: arm_pose_interface.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 30 Jun 2017 04:43:03 PM PDT
 ************************************************************************/

#ifndef _ARM_POSE_INTERFACE_H
#define _ARM_POSE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <kdl/frames.hpp>
#include <cassert>
#include <string>

namespace barrett_model
{
    /** A handle to read the cartisian state of an arm.. */
    class ArmPoseStatesHandle
    {
        public:
        ArmPoseStatesHandle() : name_(""), base_frame_(NULL), pose_(NULL), twist_(NULL)
        {};

        ArmPoseStatesHandle(const std::string& name, const std::string* base_frame, const KDL::Frame* pose, const KDL::Twist* twist) : name_(name), base_frame_(base_frame)
            {
                if (!base_frame)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Base frame pointer is null.");
                }

                if (!pose)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Pose data pointer is null.");
                }

                if (!twist)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Twist data pointer is null.");
                }

                pose_ = pose;
                twist_ = twist;
            }

            std::string getName() const {return name_;}
            std::string getBaseFrame() const {assert(base_frame_); return *base_frame_;}

            KDL::Frame getPose() const {assert(pose_); return *pose_;}
            KDL::Twist getTwist() const {assert(twist_); return *twist_;}

        private:
            std::string name_;

            const std::string* base_frame_;
            const KDL::Frame* pose_;
            const KDL::Twist* twist_;
    };

    class ArmPoseStatesInterface : public hardware_interface::HardwareResourceManager<ArmPoseStatesHandle> {};

}
#endif
