// ros
#include <ros/ros.h>
#include "lattice_planner/lattice_planner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "lattice_planner");
  ros::NodeHandle nh("~");

  planning::LatticePlanner planner(nh);

  ROS_INFO("The end of node.");

  return 0;
}
