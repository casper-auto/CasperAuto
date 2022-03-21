/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#ifndef LIBWAYPOINT_FOLLOWER_TEST_LIBWAYPOINT_FOLLOWER_H
#define LIBWAYPOINT_FOLLOWER_TEST_LIBWAYPOINT_FOLLOWER_H

#include <gtest/gtest.h>
#include "libwaypoint_follower/libwaypoint_follower.h"

enum class CoordinateResult
{
  Positive = 1,
  Negative = -1,
  Equal = 0
};

struct DirectionCheckDataSet
{
  int idx;
  double vel;
  DirectionCheckDataSet(int i, double v) :
    idx(i), vel(v)
  {}
  DirectionCheckDataSet() {}
};

struct ClosestCheckDataSet
{
  int dir;
  double vel;
  double offset;
  int num;
  geometry_msgs::PoseStamped pose;
  ClosestCheckDataSet(int d, double v, double o, int n, const geometry_msgs::PoseStamped& p)
    : dir(d), vel(v), offset(o), num(n), pose(p) {}
  ClosestCheckDataSet() {}
};

class LibWaypointFollowerTestClass
{
public:
  LibWaypointFollowerTestClass() {}
  casper_auto_msgs::WaypointArray generateWaypointArray(int driving_direction, double velocity)
  {
    return std::move(generateOffsetWaypointArray(driving_direction, velocity, 0.0, 100));
  }

  casper_auto_msgs::WaypointArray generateOffsetWaypointArray(int driving_direction, double velocity, double offset, int num)
  {
    casper_auto_msgs::WaypointArray waypoint_array;
    for (int idx = 0; idx < num; idx++)
    {
      static casper_auto_msgs::Waypoint wp;
      wp.gid = idx;
      wp.lid = idx;
      wp.pose.pose.position.x = driving_direction * (static_cast<double>(idx) + offset);
      wp.pose.pose.position.y = 0.0;
      wp.pose.pose.position.z = 0.0;
      wp.twist.twist.linear.x = velocity;
      wp.twist.twist.angular.z = 0.0;

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 0);
      quaternionTFToMsg(quaternion, wp.pose.pose.orientation);

      waypoint_array.waypoints.emplace_back(wp);
    }
    return std::move(waypoint_array);
  }

  geometry_msgs::PoseStamped generateCurrentPose(double x, double y, double yaw)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);
    quaternionTFToMsg(quaternion, pose.pose.orientation);
    return std::move(pose);
  }
};

#endif  // LIBWAYPOINT_FOLLOWER_TEST_LIBWAYPOINT_FOLLOWER_H
