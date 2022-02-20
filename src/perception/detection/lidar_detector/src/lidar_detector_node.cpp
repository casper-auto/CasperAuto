#include "lidar_detector.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_detection_node");

  LidarDetector* detector = new LidarDetector();

  ros::spin();

  return 0;
}
