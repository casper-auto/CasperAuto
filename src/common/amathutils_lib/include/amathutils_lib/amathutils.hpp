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

#ifndef AMATHUTILS_LIB_AMATHUTILS_HPP
#define AMATHUTILS_LIB_AMATHUTILS_HPP

#include <cmath>
#include <iostream>

// ROS Messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace amathutils
{

#define G_MPSS 9.80665  // m/s^2

inline double rad2deg(double _angle)
{
  return _angle * 180.0 / M_PI;
}
inline double deg2rad(double _angle)
{
  return _angle / 180.0 * M_PI;
}

inline double mps2kmph(double _mpsval)
{
  return (_mpsval * 3.6);  // mps * 60sec * 60minutes / 1000m
}

inline double kmph2mps(double _kmphval)
{
  return (_kmphval * 1000.0 / 60.0 / 60.0);  // kmph * 1000m / 60sec / 60sec
}

inline double getGravityAcceleration(double _acceleration_mpss)
{
  return _acceleration_mpss / G_MPSS;
}

inline double getAcceleration(double _v0, double _v, double _x)
{
  return (_v * _v - _v0 * _v0) / 2.0 / _x;
}

inline double getTimefromAcceleration(double _v0, double _v, double _a)
{
  return (_v - _v0) / _a;
}

bool getIntersect(double x1, double y1, double x2, double y2, double x3,
  double y3, double x4, double y4, double* intersect_x, double* intersect_y);
bool getIntersect(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3,
  geometry_msgs::Point p4, geometry_msgs::Point* intersect);

geometry_msgs::Point getNearPtOnLine(const geometry_msgs::Point &_p, const geometry_msgs::Point &_a,
                                     const geometry_msgs::Point &_b);
double find_distance(const geometry_msgs::Point &_from, const geometry_msgs::Point &_to);
double find_distance(const geometry_msgs::Pose &_from, const geometry_msgs::Pose &_to);
double find_angle(const geometry_msgs::Point &_from, const geometry_msgs::Point &_to);
bool isIntersectLine(const geometry_msgs::Point &_l1_p1, const geometry_msgs::Point &_l1_p2,
  const geometry_msgs::Point &_l2_p1, const geometry_msgs::Point &_l2_p2);
int isPointLeftFromLine(const geometry_msgs::Point &_target, const geometry_msgs::Point &_line_p1,
  const geometry_msgs::Point &_line_p2);

/**
 * @fn distanceFromSegment
 * @brief calculates the distance between from a point to closest point on a line segment
 * @param  _l1  first point of line segment        
 * @param  _l2  second point of line segment        
 * @param  _p   the point to find distance
 * @return distance between point _p and line segment(_l1,_l2) 
 */
double distanceFromSegment(const geometry_msgs::Point &_l1, const geometry_msgs::Point &_l2,
  const geometry_msgs::Point &_p);
double getPoseYawAngle(const geometry_msgs::Pose &_pose);

/**
 * @brief convert from yaw to ros-Quaternion
 * @param [in] yaw input yaw angle
 * @return quaternion
 */
geometry_msgs::Quaternion getQuaternionFromYaw(const double &_yaw);

/**
 * @brief normalize angle into [-pi to pi]
 * @param [in] _angle input angle
 * @return normalized angle
 */
double normalizeRadian(const double _angle);

double calcPosesAngleDiffRaw(const geometry_msgs::Pose &p_from, const geometry_msgs::Pose &_p_to);
double calcPosesAngleDiffDeg(const geometry_msgs::Pose &_p_from, const geometry_msgs::Pose &_p_to);
double calcPosesAngleDiffRad(const geometry_msgs::Pose &_p_from, const geometry_msgs::Pose &_p_to);

}  // namespace amathutils

#endif  // AMATHUTILS_LIB_AMATHUTILS_HPP
