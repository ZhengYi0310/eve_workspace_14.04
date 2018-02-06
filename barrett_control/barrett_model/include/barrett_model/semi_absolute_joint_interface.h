/*************************************************************************
	> File Name: semi_absolute_joint_interface.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 01 Jun 2017 10:19:31 AM PDT
 ************************************************************************/

#ifndef _SEMI_ABSOLUTE_JOINT_INTERFACE_H
#define _SEMI_ABSOLUTE_JOINT_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <angles/angles.h>

namespace barrett_model
{
    /**
     * \brieft A handle used to read and command a single joint, as well as read
     * the joint's semi-absolute resolver value 
     */
    class SemiAbsoluteJointHandle : public hardware_interface::JointHandle 
    {
        public:
            SemiAbsoluteJointHandle() {};
        SemiAbsoluteJointHandle(
            const hardware_interface::JointHandle& js,
            double resolver_range,
            double* resolver_angle,
            double* joint_offset,
            int* is_calibrated
            ) 
          : hardware_interface::JointHandle(js),
            resolver_range_(resolver_range),
            resolver_angle_(resolver_angle),
            joint_offset_(joint_offset),
            is_calibrated_(is_calibrated)
            {}

            double getResolverAngle() const 
            {
                return *resolver_angle_;
            }

            void setOffset(const double joint_offset)
            {
                *joint_offset_ = joint_offset;
            }

            double getOffset() const 
            {
                return *joint_offset_;
            }

            void setCalibrated(const int is_calibrated)
            {
                *is_calibrated_ = is_calibrated;
            }

            int isCalibrated() const 
            {
                return *is_calibrated_;
            }

            inline double getShortestDistance(double from, double to) const 
            {
                return resolver_range_/ 2.0 / M_PI * angles::shortest_angular_distance(2.0 * M_PI / resolver_range_ * from, 2.0 * M_PI / resolver_range_ * to);
            }

        private:
            double resolver_range_;
            double* resolver_angle_;
            double* joint_offset_;
            int* is_calibrated_;
    };

    /**
     * \brief Hardware interface to support commanding an array of joints 
     * \ref This HardwareInterface supports commanding the output of an array of 
     * named joints. Note that these commands can have any semantic meaning as long
     * as they each can be represented by a single double, they are not necessarily 
     * effort commands. To specify a meaning to this command, see the derived 
     * classes like \ref EffortJointInterface etc.
     */
    class SemiAbsoluteJointInterface : public hardware_interface::HardwareInterface
    {
        public:
            // Get the vector of joint names registered to this interface
            std::vector<std::string> getJointNames() const 
            {
                std::vector<std::string> out;
                out.reserve(handle_map_.size());
                for (HandleMap::const_iterator it = handle_map_.begin(); it != handle_map_.end(); it++)
                {
                    out.push_back(it->first);
                }
                return out;
            }

            /** brief Register a new joint with this interface 
             *
             * \param name The name of the new joint 
             * \param cmd A pointer to the storage for this joint's output command 
             */
            void registerJoint(const hardware_interface::JointHandle& js, double resolver_range, double* resolver_angle, double* joint_offset, int* is_calibrated)
            {
                SemiAbsoluteJointHandle handle(js, resolver_range, resolver_angle, joint_offset, is_calibrated);
                HandleMap::iterator it = handle_map_.find(js.getName());

                if (it == handle_map_.end())
                {
                    handle_map_.insert(std::make_pair(js.getName(), handle));
                }
                else
                {
                    it->second = handle;
                }
            }

            /** \brief Get a \ref JointHandle for accessing a joint's state and setting 
             * its output command 
             *
             * When a \ref JointHandle is acquired, this interface will calim the joint as a 
             * resource 
             *
             * \param name The name of the joint 
             *
             * \returns A \ref JointHandle corresponding to the joint identified by \c name 
             */
            SemiAbsoluteJointHandle getSemiAbsoluteJointHandle(const std::string& name)
            {
                HandleMap::const_iterator it = handle_map_.find(name);

                if (it == handle_map_.end())
                {
                    throw hardware_interface::HardwareInterfaceException("Could not find joint [" + name + "] in SemiAbsoluteJointInterface");

                    HardwareInterface::claim(name);
                    return it->second;
                }
            }

        protected:
            typedef std::map<std::string, SemiAbsoluteJointHandle> HandleMap;
            HandleMap handle_map_;
    };
}
#endif
