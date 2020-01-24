#include "imu_to_odom/imu_to_odom.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_to_odom_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  OdomPredictor odom_predictor(nh, nh_private);

  ros::spin();

  return 0;
}
