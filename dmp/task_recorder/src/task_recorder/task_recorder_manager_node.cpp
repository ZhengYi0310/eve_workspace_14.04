/*************************************************************************
	> File Name: task_recorder_manager_node.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Tue 27 Jun 2017 11:25:24 AM PDT
 ************************************************************************/

// system includes
// ros includes 
#include <ros/ros.h>

// local includes 
#include <task_recorder/task_recorder_manager.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TaskRecorderManager");
    ros::NodeHandle node_handle("~");

    task_recorder::TaskRecorderManager task_recorder_manager(node_handle);
    if (!task_recorder_manager.initialize())
    {
        ROS_ERROR("Could not initialize task recorder manager.");
        return -1;
    }

    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}

