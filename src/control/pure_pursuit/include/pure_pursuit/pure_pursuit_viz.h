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

#ifndef PURE_PURSUIT_PURE_PURSUIT_VIZ_H
#define PURE_PURSUIT_PURE_PURSUIT_VIZ_H

// ROS includes
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <visualization_msgs/Marker.h>
// User defined msg
#include <casper_auto_msgs/Waypoint.h>

// C++ includes
#include <memory>
#include <vector>

// User defined includes
#include <libwaypoint_follower/libwaypoint_follower.h>

namespace waypoint_follower
{
// display the next waypoint by markers.
visualization_msgs::Marker displayNextWaypoint(geometry_msgs::Point position);
// display the next target by markers.
visualization_msgs::Marker displayNextTarget(geometry_msgs::Point target);
visualization_msgs::Marker displayExpandWaypoints(const std::vector<casper_auto_msgs::Waypoint>& waypoints, int size);

double calcRadius(geometry_msgs::Point target, geometry_msgs::Pose current_pose);

// generate the locus of pure pursuit
std::vector<geometry_msgs::Point> generateTrajectoryCircle(geometry_msgs::Point target,
                                                           geometry_msgs::Pose current_pose);
// display the locus of pure pursuit by markers.
visualization_msgs::Marker displayTrajectoryCircle(std::vector<geometry_msgs::Point> traj_circle_array);

// display the search radius by markers.
visualization_msgs::Marker displaySearchRadius(geometry_msgs::Point current_pose, double search_radius);
}  // namespace waypoint_follower

#endif  // PURE_PURSUIT_PURE_PURSUIT_VIZ_H
