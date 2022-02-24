#include <iostream>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <casper_auto_msgs/Waypoint.h>
#include <casper_auto_msgs/WaypointArray.h>

#include "replay_path_planner.h"

using namespace std;

vector<double> current_pose(3);
double current_speed;
double current_z = 0;
vector<vector<double>> global_route;
double cruise_speed = 5.0;

void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  ROS_INFO("Received the current_pose ...");

  geometry_msgs::Quaternion geo_quat = msg->pose.orientation;

  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(geo_quat, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  current_pose[0] = msg->pose.position.x;
  current_pose[1] = msg->pose.position.y;
  current_pose[2] = yaw;

  current_z = msg->pose.position.z;

  // ROS_INFO("Current pose: x: %.2f, y: %.2f, yaw: %.2f", current_pose[0], current_pose[1], current_pose[2]);
}

void currentSpeedCallback(const std_msgs::Float32::ConstPtr& msg) {
  ROS_INFO("Received the current_speed ...");
  current_speed = msg->data;
  // ROS_INFO("Current speed: %.2f", current_speed);
}

void cruiseSpeedCallback(const std_msgs::Float32::ConstPtr& msg) {
  cruise_speed = msg->data;
}

void globalRouteCallback(const nav_msgs::Path::ConstPtr& msg) {
  ROS_INFO("global route got ...");
  global_route.resize(0);
  for(int i = 0; i < msg->poses.size(); i++) {
    double x = msg->poses[i].pose.position.x;
    double y = msg->poses[i].pose.position.y;

    geometry_msgs::Quaternion geo_quat = msg->poses[i].pose.orientation;
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(geo_quat, quat);
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // skip overlapped points
    if(global_route.size() == 0) {
      global_route.push_back({x, y, yaw});
    }
    else if ((global_route.size() > 0)
        && !(abs(x - global_route[global_route.size()-1][0]) < 0.001
             && abs(y - global_route[global_route.size()-1][1]) < 0.001)) {
      global_route.push_back({x, y, yaw});
    }
  }
  ROS_INFO("Received %d valid points in the global route.", int(global_route.size()));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "replay_path_planner");
  ros::NodeHandle n("~");

  ros::Subscriber current_pose_sub = n.subscribe("/current_pose", 10, currentPoseCallback);
  ros::Subscriber current_speed_sub = n.subscribe("/current_speed", 10, currentSpeedCallback);
  ros::Subscriber global_route_sub = n.subscribe("/global_route", 10, globalRouteCallback);
  ros::Subscriber cruise_speed_sub = n.subscribe("/cruise_speed", 10, cruiseSpeedCallback);

  ros::Publisher final_path_pub = n.advertise<nav_msgs::Path>("/final_path", 10);
  ros::Publisher final_waypoints_pub = n.advertise<casper_auto_msgs::WaypointArray>("/final_waypoints", 10);
  ros::Publisher final_path_marker_pub = n.advertise<visualization_msgs::Marker>("/final_path_marker", 10);

  ros::Rate rate(60);

  double lookahead_distance = 20;

  ReplayPathPlanner replay_path_planner(lookahead_distance);

  double prev_timestamp = ros::Time::now().toSec();
  vector<vector<double>> final_path = replay_path_planner.get_replayed_path(current_pose, global_route);

  while(ros::ok()) {
    ////////////////////////////////////////////////////////////////////////////
    // Run Replay Path Planner every single second
    ////////////////////////////////////////////////////////////////////////////
    double current_timestamp = ros::Time::now().toSec();
    if(current_timestamp - prev_timestamp > 1.0 && !global_route.empty()) {
      final_path = replay_path_planner.get_replayed_path(current_pose, global_route);
      // final_path = replay_path_planner.get_interpolated_path(final_path);
      prev_timestamp = current_timestamp;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Publish the ROS message containing the waypoints to the controller
    ////////////////////////////////////////////////////////////////////////////
    casper_auto_msgs::WaypointArray waypoints;
    waypoints.header.frame_id = "map";
    for(int i = 0; i < final_path.size(); i++) {
      casper_auto_msgs::Waypoint wp;
      wp.pose.pose.position.x = final_path[i][0];
      wp.pose.pose.position.y = final_path[i][1];
      wp.twist.twist.linear.x = cruise_speed;
      waypoints.waypoints.push_back(wp);
    }

    final_waypoints_pub.publish(waypoints);

    ////////////////////////////////////////////////////////////////////////////
    // Publish the ROS message containing the path
    ////////////////////////////////////////////////////////////////////////////
    nav_msgs::Path path;
    path.header.frame_id = "map";
    // path.header.stamp = ros::Time::now();
    for(int i = 0; i < final_path.size(); i++) {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = final_path[i][0];
      pose.pose.position.y = final_path[i][1];
      pose.pose.position.z = current_z + 1.0;

      geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(final_path[i][2]);
      pose.pose.orientation = quat;
      path.poses.push_back(pose);
    }

    final_path_pub.publish(path);

    ////////////////////////////////////////////////////////////////////////////
    // Construct and Publish generated paths as visualization markers
    ////////////////////////////////////////////////////////////////////////////
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "map";
    // points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "replayed_path";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w= line_list.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;
    line_list.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    // Create the vertices for the points and lines
    for(int i = 0; i < final_path.size(); i++) {
      geometry_msgs::Point p;
      p.x = final_path[i][0];
      p.y = final_path[i][1];
      p.z = 0.0;

      points.points.push_back(p);
      line_strip.points.push_back(p);
      line_list.points.push_back(p);
      p.z += 1.0;
      line_list.points.push_back(p);
    }

    final_path_marker_pub.publish(points);
    // final_path_marker_pub.publish(line_strip);
    // final_path_marker_pub.publish(line_list);

    ////////////////////////////////////////////////////////////////////////////
    // ROS Spin
    ////////////////////////////////////////////////////////////////////////////
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("The iteration end.");
  }

  return 0;
}
