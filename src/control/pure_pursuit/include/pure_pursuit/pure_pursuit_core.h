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

#ifndef PURE_PURSUIT_PURE_PURSUIT_CORE_H
#define PURE_PURSUIT_PURE_PURSUIT_CORE_H

// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

// User defined includes
#include <autoware_config_msgs/ConfigWaypointFollower.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <casper_auto_msgs/Waypoint.h>
#include <casper_auto_msgs/WaypointArray.h>
#include <autoware_health_checker/health_checker/health_checker.h>
#include <pure_pursuit/pure_pursuit.h>

// C++ includes
#include <vector>
#include <memory>
#include <string>

namespace waypoint_follower
{
enum class Mode : int32_t
{
  waypoint,
  dialog,
  unknown = -1,
};

template <class T>
typename std::underlying_type<T>::type enumToInteger(T t)
{
  return static_cast<typename std::underlying_type<T>::type>(t);
}

class PurePursuitNode
{
public:
  PurePursuitNode();
  ~PurePursuitNode() = default;

  void run();
  friend class PurePursuitNodeTestSuite;

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::shared_ptr<autoware_health_checker::HealthChecker> health_checker_ptr_;

  // class
  PurePursuit pp_;

  // publisher
  ros::Publisher pub1_, pub2_, pub11_, pub12_, pub13_, pub14_, pub15_, pub16_, pub17_, pub18_;

  // subscriber
  ros::Subscriber sub1_, sub2_, sub3_, sub4_;

  // control loop update rate
  double update_rate_;

  // variables
  bool is_linear_interpolation_, add_virtual_end_waypoints_;
  bool is_waypoint_set_, is_pose_set_, is_velocity_set_;
  double current_linear_velocity_, command_linear_velocity_;
  double wheel_base_;
  int expand_size_;
  WaypointArrayDirection direction_;
  int32_t velocity_source_;          // 0 = waypoint, 1 = Dialog
  double const_lookahead_distance_;  // meter
  double const_velocity_;            // km/h
  double lookahead_distance_ratio_;
  // the next waypoint must be outside of this threshold.
  double minimum_lookahead_distance_;
  std::string output_interface_;

  // callbacks
  void callbackFromConfig(const autoware_config_msgs::ConfigWaypointFollowerConstPtr& config);
  void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
  void callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg);
  void callbackFromWayPoints(const casper_auto_msgs::WaypointArrayConstPtr& msg);

  // initializer
  void initForROS();

  // functions
  void publishControlCommands(const bool& can_get_curvature, const double& kappa) const;
  void publishTwistStamped(const bool& can_get_curvature, const double& kappa) const;
  void publishCtrlCmdStamped(const bool& can_get_curvature, const double& kappa) const;
  void publishDeviationCurrentPosition(const geometry_msgs::Point& point,
                                       const std::vector<casper_auto_msgs::Waypoint>& waypoints) const;
  void connectVirtualLastWaypoints(casper_auto_msgs::WaypointArray* expand_waypoint_array, WaypointArrayDirection direction);

  int getSgn() const;
  double computeLookaheadDistance() const;
  double computeCommandVelocity() const;
  double computeCommandAccel() const;
  double computeAngularGravity(double velocity, double kappa) const;
};

double convertCurvatureToSteeringAngle(const double& wheel_base, const double& kappa);

inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}

}  // namespace waypoint_follower

#endif  // PURE_PURSUIT_PURE_PURSUIT_CORE_H
