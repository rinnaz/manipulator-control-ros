#include "kuka_lbr_iiwa_control/iiwa_controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iiwa_controller");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_WARN("iiwa_controller process started");

  IiwaController controller;

  // std::vector<double> goal {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  // controller.setJointStates(goal);
  // controller.printCurrentJointState();

  // controller.setDefaultPose();

  while(ros::ok())
  {
    controller.updateKinematicState();
    controller.printCurrentJointState();
    controller.updateEePose();
    ros::Duration(1.0).sleep();
  }

  ros::waitForShutdown();
  return 0;
}
 
