/*************************************************************************
	> File Name: test.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 13 Dec 2017 12:13:43 PM PST
 ************************************************************************/

#include <iostream>
#include <ros/ros.h>
#include "ops_wbc_digital_filter/differentiator.hpp"
#include "ops_wbc_digital_filter/pseudo_differentiator.hpp"
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ops_wbc_digital_filter");
    ros::NodeHandle nh;

    using namespace ops_wbc_digital_filter;
    Eigen::VectorXd q(1), dq(1);
    double period = 0.001;
    double cutoff_freq = 30.0;

    DifferentiatorPtr differentiator = std::make_shared<PseudoDifferentiator>(period, cutoff_freq);
    differentiator->init(q, dq);

    ros::Rate r(1.0 / period);
    double t = 0.0;
    double f = 1.0;

    Eigen::VectorXd pre_q = Eigen::VectorXd::Zero(1);

    while(ros::ok())
    {
        q = Eigen::VectorXd::Constant(1, sin(2.0 * M_PI * f * t));
        differentiator->apply(q);
        std::cout << "new dq: " << std::endl << dq << std::endl;
        std::cout << "answer2: " << 1.0 / period * (q - pre_q) << std::endl;
        pre_q = q;
        t += period;
        r.sleep();
    }

    return 0;
}

