#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

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
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <odr_map.h>

double distance2D(geometry_msgs::Point a, geometry_msgs::Point b);

class RoutePlanner {
public:
  RoutePlanner(ros::NodeHandle nh, std::shared_ptr<OdrMap> map_parser,
               double route_distance = 100, double plan_cycle = 10);

private:
  ros::NodeHandle nh_;

  // ros timer
  ros::Timer timer_;

  double prev_timestamp_;

  // ros publishers
  ros::Publisher global_route_pub_;

  // ros subscribers
  ros::Subscriber current_pose_sub_;
  ros::Subscriber waypoints_sub_;
  ros::Subscriber goal_sub_;

  // msg
  nav_msgs::Path waypoints_;
  bool waypoints_defined_;
  geometry_msgs::Pose goal_;
  bool goal_defined_;
  nav_msgs::Path global_route_;

  // map_parser
  std::shared_ptr<OdrMap> map_parser_;

  // map objects
  std::vector<casper_auto_msgs::MapBasepath> map_paths_;

  bool current_pose_initialized_;

  //
  geometry_msgs::Point start_point_;
  geometry_msgs::Point target_point_;

  //
  double route_distance_;
  double plan_cycle_; // 10 s for example

  void initalize();

  void planRoute();

  void timerCallback(const ros::TimerEvent &e);

  void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  void waypointsCallback(const nav_msgs::Path::ConstPtr &msg);

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  int getNearestPathIdx(geometry_msgs::Point &query_point);

  int getClosestIndex(nav_msgs::Path waypoints, geometry_msgs::Point point);

  int getClosestIndex(std::vector<geometry_msgs::Point> points,
                      geometry_msgs::Point point);

  geometry_msgs::Point getTargetPoint(nav_msgs::Path waypoints,
                                      double route_distance);

  std::vector<geometry_msgs::Point>
  pathSearch(geometry_msgs::Point &start_point,
             geometry_msgs::Point &target_point);

  std::vector<geometry_msgs::Point>
  convertGeometryPoints(std::vector<geometry_msgs::Point32> &points);

  std::vector<int> reconstructPath(int curr,
                                   std::unordered_map<int, int> &parent);

  nav_msgs::Path convertToNavPathMsg(std::vector<geometry_msgs::Point> &points);
};

#endif /* ROUTE_PLANNER_H */
