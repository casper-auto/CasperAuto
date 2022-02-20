#include "driveway_object_filter/driveway_object_filter.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "driveway_object_filter_node");

  DrivewayObjectFilter object_filter;

  ros::spin();

  return 0;
}
