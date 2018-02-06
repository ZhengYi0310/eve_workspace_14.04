#include <wam_dmp_controller/dmp_joint_position_controller.h>
#include <wam_dmp_controller/dmp_ik_controller.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_wam_dmp_controller");
  wam_dmp_controller::DMPJointPositionController joint_controller;
  //wam_dmp_controller::DMPIKController ik_controller;
  return 0;
}
