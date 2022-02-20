// ros
#include <ros/ros.h>
#include "spiral_planner/spiral_planner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "spiral_planner");
  ros::NodeHandle nh("~");

  planning::SpiralPlanner spiral(nh);

  ROS_INFO("The end of node.");

  return 0;
}
