/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include <barrett_hw/wam_server.h>
#include <iostream>


using namespace KDL;
using namespace std;
using namespace urdf;
using namespace terse_roscpp;

void printLink(const SegmentMap::const_iterator& link, const std::string& prefix)
{
  cout << prefix << "- Segment " << GetTreeElementSegment(link->second).getName() << " has "
       << GetTreeElementChildren(link->second).size() << " children" << endl;
  for (unsigned int i=0; i < GetTreeElementChildren(link->second).size(); i++)
      printLink(GetTreeElementChildren(link->second)[i], prefix + "  ");
}


int main(int argc, char** argv)
{
    /*
    if (argc < 2){
        std::cerr << "Expect xml file to parse" << std::endl;
        return -1;
    }
    */
    // Set up real time task 
    mlockall(MCL_CURRENT | MCL_FUTURE);
    // initialize ROS 
    ros::init(argc, argv, "kdl_parser_check", ros::init_options::NoSigintHandler);
    
    // Add custom signal handlers 
    signal(SIGTERM, quitRequested);
    signal(SIGINT, quitRequested);
    signal(SIGHUP, quitRequested);    

    ros::NodeHandle barrett_nh("barrett");
    const std::string product_name = "wam_left";
    ros::NodeHandle product_nh(barrett_nh, "products/" + product_name);
    barrett_hw::BarrettHW barrett_robot(barrett_nh);
    
    /**************/
    //barrett_robot.configure();
    /***************/
    
    std::string urdf_str;
    std::string urdf_path;
    std::string root_joint_name;
    std::string tip_joint_name;
    param::require(barrett_nh, "urdf_file_path", urdf_path, "The path to the Barrett Wam URDF file.");
    param::require(barrett_nh, "robot_description_yi", urdf_str, "The URDF for the Barrett Wam arm.");
    param::require(product_nh, "tip_segment", tip_joint_name, "WAM tip joint in URDF.");
    param::require(product_nh, "root_segment", root_joint_name, "WAM root joint in URDF.");

    ROS_INFO("the urdf file path %s", urdf_path.c_str());
    ROS_INFO("the urdf string %s", urdf_str.c_str());

    Model robot_model;
    if (!robot_model.initFile(urdf_path))
    {cerr << "Could not generate robot model" << endl; return false;}

    Tree my_tree;
    if (!kdl_parser::treeFromUrdfModel(robot_model, my_tree)) 
    {cerr << "Could not extract kdl tree" << endl; return false;}

    // walk through tree
    cout << " ======================================" << endl;
    cout << " Tree has " << my_tree.getNrOfSegments() << " link(s) and a root link" << endl;
    cout << " ======================================" << endl;
    SegmentMap::const_iterator root = my_tree.getRootSegment();
    printLink(root, "");

    KDL::Chain kdl_chain;
    bool res;
    try
    {
        res = my_tree.getChain(root_joint_name, tip_joint_name, kdl_chain);
    }
    catch(...)
    {
        ROS_ERROR("Could not extract chain between %s and %s from the kdl tree.", root_joint_name.c_str(), tip_joint_name.c_str());
    }

    if (!res)
    {
        ROS_ERROR("Could not extract chain between %s and %s from kdl tree", root_joint_name.c_str(), tip_joint_name.c_str());
    }
        
        
    if (static_cast<size_t>(kdl_chain.getNrOfJoints()) != 7)
    {
        ROS_ERROR("For now, the KDL chain needs to have 7 arm joints, but only has >%lu<", static_cast<size_t>(kdl_chain.getNrOfJoints()));
    }
    else
    {
        ROS_INFO_STREAM("the KDL chain has " << kdl_chain.getNrOfJoints() << " joints!");
    }
    

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
    /*
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
    */
    
}


