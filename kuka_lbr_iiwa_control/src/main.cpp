#include "kuka_lbr_iiwa_control/iiwa_controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iiwa_controller");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_WARN("iiwa_controller process started");

  IiwaController controller;

  controller.printCurrentJointState();

  ros::shutdown();
  return 0;
}
 
