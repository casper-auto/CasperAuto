#ifndef TRAFFIC_LIGHT_PROCESSOR_H
#define TRAFFIC_LIGHT_PROCESSOR_H

#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <vector>

// ros
#include <casper_auto_msgs/CenterLanes.h>
#include <casper_auto_msgs/MapBasepath.h>
#include <casper_auto_msgs/MapBoundary.h>
#include <casper_auto_msgs/MapCrosswalk.h>
#include <casper_auto_msgs/MapLane.h>
#include <casper_auto_msgs/MapObject.h>
#include <casper_auto_msgs/MapStopLine.h>
#include <casper_auto_msgs/MapTrafficLight.h>
#include <casper_auto_msgs/VehicleState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <odr_map.h>

double distance2D(geometry_msgs::Point a, geometry_msgs::Point b);

class TrafficLightProcessor {
public:
  TrafficLightProcessor(ros::NodeHandle nh, std::shared_ptr<OdrMap> map_parser);

private:
  ros::NodeHandle nh_;

  // ros timer
  ros::Timer timer_;

  // ros subscribers
  ros::Subscriber current_pose_sub_;
  ros::Subscriber selected_path_sub_;
  ros::Subscriber traffic_light_state_sub_;

  // ros publishers
  ros::Publisher stop_point_pub_;
  ros::Publisher stop_line_marker_pub_;
  ros::Publisher stop_line_activated_pub_;
  ros::Publisher multi_lane_flag_pub_;

  // map_parser
  std::shared_ptr<OdrMap> map_parser_;

  // map objects
  std::vector<casper_auto_msgs::MapBasepath> map_paths_;
  std::vector<casper_auto_msgs::MapLane> map_lane_boundaries_;

  geometry_msgs::Point current_position_;
  std::vector<geometry_msgs::Pose> selected_path_;
  bool current_pose_initialized_;

  bool found_stop_line_;
  geometry_msgs::Point stop_point_;
  bool multi_lane_flag_;

  int traffic_light_state_;

  void initalize();

  void timerCallback(const ros::TimerEvent &e);

  void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  void selectedPathCallback(const nav_msgs::Path::ConstPtr &msg);

  void trafficLightStateCallback(const std_msgs::Int32::ConstPtr &msg);

  int getNearestPathIdx(geometry_msgs::Point &query_point);
  std::vector<int> getNearestLaneIdx(geometry_msgs::Point &query_point);
};

#endif /* TRAFFIC_LIGHT_PROCESSOR_H */
