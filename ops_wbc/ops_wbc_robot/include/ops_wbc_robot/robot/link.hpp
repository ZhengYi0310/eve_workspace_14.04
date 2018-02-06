#ifndef __OPS_WBC_ROBOT_LINK_HPP
#define __OPS_WBC_ROBOT_LINK_HPP

#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include "ops_wbc_robot/robot/transformation.hpp"

namespace ops_wbc_robot
{
    class Link
    {
        public:
            explicit Link()
            {
                T_org = Eigen::Matrix4d::Identity();
                C = Eigen::Vector3d::Zero();
                I = Eigen::Matrix3d::Zero();
            }

            void print()
            {
                std::cout << "name: "       << name       << std::endl 
                          << "joint_type: " << joint_type << std::endl
                          << "parent: "     << parent   << std::endl
                          << "child: "      << child      << std::endl
                          << "ep: "         << ep         << std::endl
                          << "T_org: "      << std::endl  << T_org << std::endl
                          << "C: "          << std::endl  << C << std::endl
                          << "m: "          << m          << std::endl
                          << "I: "          << std::endl  << I << std::endl
                          << "q_min: "      << q_min      << std::endl
                          << "q_max: "      << q_max      << std::endl
                          << "dq_max: "     << dq_max     << std::endl
                          << "tau: "        << tau        << std::endl
                          << "tau_max: "    << tau_max    << std::endl;
            }

            std::string name = "";
            std::string joint_type = "";
            std::string parent = "";
            std::string child = "";
            bool ep = false;

            TransformationPtr = nullptr;

            Eigen::Matrix4d T_org;
            Eigen::Vector3d C;

            double m = 0.0;
            Eigen::Matrix3d I;

            double q_min = 0.0;
            double q_max = 0.0;
            double dq_max = 0.0;
            double tau = 0.0;
            double tau_max = 0.0;
    };
    using LinkPtr = std::shared_ptr<Link>;
}

#endif // __OPS_WBC_ROBOT_LINK_HPP
