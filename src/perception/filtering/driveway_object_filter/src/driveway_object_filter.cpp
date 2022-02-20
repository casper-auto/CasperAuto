/*
 *  driveway_object_filter.cpp
 *
 *  Created on: October, 23rd, 2018
 */

#include "driveway_object_filter/driveway_object_filter.h"

std::vector<double> rotate(const std::vector<double> &in, const double theta) {
  std::vector<double> out(2);
  double s = sin(theta);
  double c = cos(theta);
  out[0] = in[0] * c - in[1] * s;
  out[1] = in[0] * s + in[1] * c;
  return out;
}

std::vector<double> globalToLocal(const std::vector<double> &center,
                                  const double theta,
                                  const std::vector<double> &p) {
  std::vector<double> delta(2);
  delta[0] = p[0] - center[0];
  delta[1] = p[1] - center[1];
  return rotate(delta, -theta);
}

std::vector<double> localToGlobal(const std::vector<double> &center,
                                  const double theta,
                                  const std::vector<double> &p) {
  std::vector<double> out = rotate(p, theta);
  out[0] += center[0];
  out[1] += center[1];
  return out;
}

DrivewayObjectFilter::DrivewayObjectFilter() : nh_("") {
  ////////////////////////////////////////////////////////////////////////////
  // Wait for opendrive map msg
  ////////////////////////////////////////////////////////////////////////////
  boost::shared_ptr<std_msgs::String const> shared_map_msg =
      ros::topic::waitForMessage<std_msgs::String>(
          "/carla/ego_vehicle/opendrive_map", nh_);
  std::string xml_str = shared_map_msg->data;

  /////////////////////////////////////////////////////////////////
  // Data type conversion and value checking
  /////////////////////////////////////////////////////////////////
  map_parser_ = std::make_shared<OdrMap>(xml_str, 1.0);
  map_initialized_ = true;

  filtered_objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(
      "/filtered_objects", 10);

  current_pose_sub_ = nh_.subscribe(
      "/current_pose", 10, &DrivewayObjectFilter::currentPoseCallback, this);

  detected_objects_sub_ =
      nh_.subscribe("/detected_objects", 10,
                    &DrivewayObjectFilter::detectedObjectsCallback, this);

  // createTimer
  timer_ = nh_.createTimer(ros::Duration(0.1),
                           &DrivewayObjectFilter::timerCallback, this);
}

void DrivewayObjectFilter::timerCallback(const ros::TimerEvent &e) {
  filtered_objects_.header.frame_id = "ego_vehicle/lidar";
  filtered_objects_.header.stamp = ros::Time::now();
  filtered_objects_pub_.publish(filtered_objects_);
}

void DrivewayObjectFilter::currentPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  // ROS_INFO("Current pose got ...");

  current_pose2d_.x = msg->pose.position.x;
  current_pose2d_.y = msg->pose.position.y;

  geometry_msgs::Quaternion quat = msg->pose.orientation;

  double quatx = msg->pose.orientation.x;
  double quaty = msg->pose.orientation.y;
  double quatz = msg->pose.orientation.z;
  double quatw = msg->pose.orientation.w;

  tf::Quaternion q(quatx, quaty, quatz, quatw);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current_pose2d_.yaw = yaw;
}

void DrivewayObjectFilter::detectedObjectsCallback(
    const autoware_msgs::DetectedObjectArray::ConstPtr &msg) {
  if (map_initialized_) {
    filtered_objects_.objects.resize(0);
    for (autoware_msgs::DetectedObject obj : msg->objects) {
      geometry_msgs::Point point = obj.pose.position;

      // filter out the points from ego
      if(point.x > -2.5 && point.x < 2.5 && point.y > -1.0 && point.y < 1.0) {
        continue;
      }

      // filter out those are too big
      geometry_msgs::Vector3 dimensions = obj.dimensions;
      if(dimensions.x * dimensions.y > 15) {
        continue;
      }

      std::vector<double> point_global = localToGlobal(
          std::vector<double>({current_pose2d_.x, current_pose2d_.y}),
          current_pose2d_.yaw, std::vector<double>({point.x, point.y}));

      if (map_parser_->isInRoad(point_global[0], point_global[1], 0)) {
        filtered_objects_.objects.push_back(obj);
      }
    }
  } else {
    filtered_objects_ = *msg;
  }
}
