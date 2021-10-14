#include "kuka_lbr_iiwa_detector/kuka_lbr_iiwa_detector.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_detector");

  ROS_WARN("kuka_lbr_iiwa_detector process started");

  MarkerDetector *detector = new MarkerDetector();

  ros::spin();
  return 0;
}
 
