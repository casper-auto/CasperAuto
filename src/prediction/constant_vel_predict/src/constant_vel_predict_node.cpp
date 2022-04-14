#include <constant_vel_predict/constant_vel_predict.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "constant_vel_predict");

  AgentsEstimation estimator;

  while (ros::ok()) {
    ros::spin();
  }

  return 0;
}
