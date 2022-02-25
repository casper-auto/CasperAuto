/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LIBWAYPOINT_FOLLOWER_LIBWAYPOINT_FOLLOWER_H
#define LIBWAYPOINT_FOLLOWER_LIBWAYPOINT_FOLLOWER_H

#define EIGEN_MPL2_ONLY
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>

// C++ header
#include <iostream>
#include <sstream>
#include <fstream>
#include <utility>
#include <vector>

// ROS header
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <casper_auto_msgs/WaypointArray.h>

constexpr double ERROR = 1e-6;

enum class WaypointArrayDirection : int
{
  Backward = -1,
  Forward = 1,
  Error = 0
};

class WayPoints
{
protected:
  casper_auto_msgs::WaypointArray current_waypoints_;

public:
  void setPath(const casper_auto_msgs::WaypointArray& waypoints)
  {
    current_waypoints_ = waypoints;
  }
  int getSize() const;
  bool isEmpty() const
  {
    return current_waypoints_.waypoints.empty();
  };
  double getInterval() const;
  geometry_msgs::Point getWaypointPosition(int waypoint) const;
  geometry_msgs::Quaternion getWaypointOrientation(int waypoint) const;
  geometry_msgs::Pose getWaypointPose(int waypoint) const;
  double getWaypointVelocityMPS(int waypoint) const;
  casper_auto_msgs::WaypointArray getCurrentWaypoints() const
  {
    return current_waypoints_;
  }
  bool inDrivingDirection(int waypoint, geometry_msgs::Pose current_pose) const;
};

// inline function (less than 10 lines )
inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}
inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}

tf::Vector3 point2vector(geometry_msgs::Point point);                         // convert point to vector
geometry_msgs::Point vector2point(tf::Vector3 vector);                        // convert vector to point
tf::Vector3 rotateUnitVector(tf::Vector3 unit_vector, double degree);         // rotate unit vector by degree
geometry_msgs::Point rotatePoint(geometry_msgs::Point point, double degree);  // rotate point vector by degree

double DecelerateVelocity(double distance, double prev_velocity);
geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point,
                                            geometry_msgs::Pose current_pose);  // transform point into the coordinate
                                                                                // of current_pose
geometry_msgs::Point calcAbsoluteCoordinate(geometry_msgs::Point point,
                                            geometry_msgs::Pose current_pose);  // transform point into the global
                                                                                // coordinate
double getPlaneDistance(geometry_msgs::Point target1,
                        geometry_msgs::Point target2);  // get 2 dimentional distance between target 1 and target 2
WaypointArrayDirection getWaypointArrayDirection(const casper_auto_msgs::WaypointArray& current_path);
WaypointArrayDirection getWaypointArrayDirectionByPosition(const casper_auto_msgs::WaypointArray& current_path);
WaypointArrayDirection getWaypointArrayDirectionByVelocity(const casper_auto_msgs::WaypointArray& current_path);
int getClosestWaypoint(const casper_auto_msgs::WaypointArray& current_path, geometry_msgs::Pose current_pose);
bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double* a, double* b, double* c);
double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double sa, double b, double c);
double getRelativeAngle(geometry_msgs::Pose waypoint_pose, geometry_msgs::Pose vehicle_pose);
double calcCurvature(const geometry_msgs::Point &target, const geometry_msgs::Pose &curr_pose);
double calcDistSquared2D(const geometry_msgs::Point &p, const geometry_msgs::Point &q);
double calcLateralError2D(const geometry_msgs::Point &a_start, const geometry_msgs::Point &a_end,
                          const geometry_msgs::Point &b);
double calcRadius(const geometry_msgs::Point &target, const geometry_msgs::Pose &current_pose);
std::vector<geometry_msgs::Pose> extractPoses(const casper_auto_msgs::WaypointArray &waypoint_array);
std::vector<geometry_msgs::Pose> extractPoses(const std::vector<casper_auto_msgs::Waypoint> &wps);
std::pair<bool, int32_t> findClosestIdxWithDistAngThr(const std::vector<geometry_msgs::Pose> &curr_ps,
                                                      const geometry_msgs::Pose &curr_pose,
                                                      double dist_thr = 3.0,
                                                      double angle_thr = M_PI_2);
geometry_msgs::Quaternion getQuaternionFromYaw(const double &_yaw);
bool isDirectionForward(const std::vector<geometry_msgs::Pose> &poses);
double normalizeEulerAngle(double euler);
geometry_msgs::Point transformToAbsoluteCoordinate2D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &current_pose);
geometry_msgs::Point transformToAbsoluteCoordinate3D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &origin);
geometry_msgs::Point transformToRelativeCoordinate2D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &current_pose);
geometry_msgs::Point transformToRelativeCoordinate3D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &current_pose);

#endif  // LIBWAYPOINT_FOLLOWER_LIBWAYPOINT_FOLLOWER_H
