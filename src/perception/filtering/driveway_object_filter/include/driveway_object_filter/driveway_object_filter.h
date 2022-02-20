/*
 *  driveway_object_filter.h
 *
 *  Created on: October, 23rd, 2018
 */

#ifndef DRIVEWAY_OBJECT_FILTER_H
#define DRIVEWAY_OBJECT_FILTER_H

#include <iostream>
#include <string>
#include <vector>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <tf/transform_datatypes.h>

#include <odr_map.h>

class Pose2D {
public:
  double x, y, yaw;

  Pose2D() : x(0), y(0), yaw(0) {}

  Pose2D(double px, double py, double heading) : x(px), y(py), yaw(heading) {}
};

class DrivewayObjectFilter {
public:
  DrivewayObjectFilter();

  ~DrivewayObjectFilter() {}

private:
  ros::NodeHandle nh_;

  // ros timer
  ros::Timer timer_;

  ros::Publisher filtered_objects_pub_;

  ros::Subscriber current_pose_sub_;
  ros::Subscriber detected_objects_sub_;

  Pose2D current_pose2d_;
  autoware_msgs::DetectedObjectArray detected_objects_;
  autoware_msgs::DetectedObjectArray filtered_objects_;

  std::shared_ptr<OdrMap> map_parser_;
  bool map_initialized_;

  void timerCallback(const ros::TimerEvent &e);

  void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  void detectedObjectsCallback(
      const autoware_msgs::DetectedObjectArray::ConstPtr &msg);
};

#endif // DRIVEWAY_OBJECT_FILTER_H
