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

#include "libwaypoint_follower/pure_pursuit.h"
#include "libwaypoint_follower/libwaypoint_follower.h"

#include <limits>
#include <utility>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

void PurePursuit::setUseLerp(bool ul) { use_lerp_ = ul; }

void PurePursuit::setLookaheadDistance(double ld) { lookahead_distance_ = ld; }

void PurePursuit::setClosestThreshold(double clst_thr_dist, double clst_thr_ang)
{
  clst_thr_dist_ = clst_thr_dist;
  clst_thr_ang_ = clst_thr_ang;
}

geometry_msgs::Point PurePursuit::getLocationOfNextWaypoint() { return loc_next_wp_; }

geometry_msgs::Point PurePursuit::getLocationOfNextTarget() { return loc_next_tgt_; }

bool PurePursuit::isRequirementsSatisfied()
{
  return (curr_wps_ptr_  && curr_pose_ptr_) ? true : false;
}

std::pair<bool, double> PurePursuit::run()
{
  std::pair<bool, double> error = std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
  if (!isRequirementsSatisfied())
    return error;

  auto clst_pair = findClosestIdxWithDistAngThr(*curr_wps_ptr_, *curr_pose_ptr_, clst_thr_dist_, clst_thr_ang_);

  if (!clst_pair.first)
  {
    ROS_WARN("cannot find, curr_bool: %d, clst_idx: %d", clst_pair.first, clst_pair.second);
    return error;
  }

  int32_t next_wp_idx = findNextPointIdx(clst_pair.second);
  if (next_wp_idx == -1)
  {
    ROS_WARN("lost next waypoint");
    return error;
  }

  loc_next_wp_ = curr_wps_ptr_->at(next_wp_idx).position;

  geometry_msgs::Point next_tgt_pos;
  // if use_lerp_ is false or next waypoint is first
  if (!use_lerp_ || next_wp_idx == 0)
  {
    next_tgt_pos = curr_wps_ptr_->at(next_wp_idx).position;
  }
  else
  {
    // linear interpolation
    std::pair<bool, geometry_msgs::Point> lerp_pair = lerpNextTarget(next_wp_idx);

    if (!lerp_pair.first)
    {
      ROS_WARN("lost target! ");
      return error;
    }

    next_tgt_pos = lerp_pair.second;
  }
  loc_next_tgt_ = next_tgt_pos;

  double kappa = calcCurvature(next_tgt_pos, *curr_pose_ptr_);

  return std::make_pair(true, kappa);
}

// linear interpolation of next target
std::pair<bool, geometry_msgs::Point> PurePursuit::lerpNextTarget(int32_t next_wp_idx)
{
  std::pair<bool, geometry_msgs::Point> error = std::make_pair(false, geometry_msgs::Point());
  constexpr double ERROR2 = 1e-5;  // 0.00001
  const geometry_msgs::Point &vec_end = curr_wps_ptr_->at(next_wp_idx).position;
  const geometry_msgs::Point &vec_start = curr_wps_ptr_->at(next_wp_idx - 1).position;
  const geometry_msgs::Pose &curr_pose = *curr_pose_ptr_;

  Eigen::Vector3d vec_a((vec_end.x - vec_start.x),
                        (vec_end.y - vec_start.y),
                        (vec_end.z - vec_start.z));

  if (vec_a.norm() < ERROR2)
    return error;

  double lateral_error = calcLateralError2D(
      vec_start, vec_end, curr_pose.position);

  if (std::fabs(lateral_error) > lookahead_distance_)
    return error;

  /* calculate the position of the foot of a perpendicular line */
  Eigen::Vector2d uva2d(vec_a.x(), vec_a.y());
  uva2d.normalize();
  Eigen::Rotation2Dd rot = (lateral_error > 0) ? Eigen::Rotation2Dd(-M_PI / 2.0) : Eigen::Rotation2Dd(M_PI / 2.0);
  Eigen::Vector2d uva2d_rot = rot * uva2d;

  geometry_msgs::Point h;
  h.x = curr_pose.position.x + fabs(lateral_error) * uva2d_rot.x();
  h.y = curr_pose.position.y + fabs(lateral_error) * uva2d_rot.y();
  h.z = curr_pose.position.z;

  // if there is a intersection
  if (fabs(fabs(lateral_error) - lookahead_distance_) < ERROR2)
  {
    return std::make_pair(true, h);
  }
  else
  {
    // if there are two intersection
    // get intersection in front of vehicle
    double s = sqrt(pow(lookahead_distance_, 2) - pow(lateral_error, 2));
    geometry_msgs::Point res;
    res.x = h.x + s * uva2d.x();
    res.y = h.y + s * uva2d.y();
    res.z = curr_pose.position.z;
    return std::make_pair(true, res);
  }
}

int32_t PurePursuit::findNextPointIdx(int32_t search_start_idx)
{
  // if waypoints are not given, do nothing.
  if (curr_wps_ptr_->size() < 3 || search_start_idx == -1)
    return -1;

  // look for the next waypoint.
  for (int32_t i = search_start_idx; i < static_cast<int32_t>(curr_wps_ptr_->size()); i++)
  {
    // if search waypoint is the last
    if (i == static_cast<int32_t>(curr_wps_ptr_->size() - 1))
    {
      return i;
    }

    // if waypoint is  not on the front
    if (isDirectionForward(*curr_wps_ptr_))
    {
      if (transformToRelativeCoordinate2D(curr_wps_ptr_->at(i).position, *curr_pose_ptr_).x < 0)
        continue;
    }
    else
    {
      if (transformToRelativeCoordinate2D(curr_wps_ptr_->at(i).position, *curr_pose_ptr_).x > 0)
        continue;
    }

    const geometry_msgs::Point &curr_motion_point = curr_wps_ptr_->at(i).position;
    const geometry_msgs::Point &curr_pose_point = curr_pose_ptr_->position;
    // if there exists an effective waypoint
    const double ds = calcDistSquared2D(curr_motion_point, curr_pose_point);
    if (ds > std::pow(lookahead_distance_, 2))
      return i;
  }

  // if this program reaches here , it means we lost the waypoint!
  ROS_ERROR("lost waypoint!");
  return -1;
}

void PurePursuit::setCurrentPose(const geometry_msgs::Pose &msg)
{
  curr_pose_ptr_ = std::make_shared<geometry_msgs::Pose>();
  *curr_pose_ptr_ = msg;
}

void PurePursuit::setWaypoints(const std::vector<geometry_msgs::Pose> &msg)
{
  curr_wps_ptr_ = std::make_shared<std::vector<geometry_msgs::Pose>>();
  *curr_wps_ptr_ = msg;
}

