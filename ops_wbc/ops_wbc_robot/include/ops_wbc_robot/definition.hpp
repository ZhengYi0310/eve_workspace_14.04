/////////////////////////////////////////////////
/// \file definition.hpp
/// \brief Declare static const values
/// \author Yi Zheng
/////////////////////////////////////////////////

#ifndef __OPS_WBC_ROBOT_DEFINITION_HPP
#define __OPS_WBC_ROBOT_DEFINITION_HPP

#include <string>

namespace ops_wbc_robot
{
    namespace joint
    {
        static const std::string REVOLUTE_X = "revolute_x";
        static const std::string REVOLUTE_Y = "revolute_y";
        static const std::string REVOLUTE_Z = "revolute_z";
        static const std::string PRISMATIC_X = "prismatic_x";
        static const std::string PRISMATIC_Y = "prismatic_y";
        static const std::string PRISMATIC_Z = "prismatic_z";
        static const std::string FIXED = "fixed";
    }

    namespace frame 
    {
        static const std::string WORLD = "map";
    }

    namespace mobility
    {
        enum Type : uint8_t
        {
            SENTRY_LOWER,
            FIXED,
            MOBILITY_2D,
            MOBILITY_3D,
            SENTRY_UPPER,
        };

        namespace type
        {
            static const std::string MECANUM_WHEEL = "mecanum_wheel";
        }
    } 
}

#endif // __OPS_WBC_ROBOT_DEFINITION_HPP
