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
 *
 *  Created on: Apr 8, 2015
 *      Author: sujiwo
 */

#ifndef LIBVECTORMAP_MATH_H
#define LIBVECTORMAP_MATH_H

#include <Eigen/Eigen>
#include <cmath>

typedef Eigen::Vector2f Point2;
typedef Eigen::Vector3f Point3;
typedef Eigen::Vector3f Vector3;
typedef Eigen::Vector4f Point4;
typedef Eigen::Vector4f Vector4;
typedef Eigen::Vector2f Vector2;
typedef Eigen::Quaternionf Quaternion;
typedef Eigen::Matrix4f Matrix4;
typedef Eigen::Matrix2f Matrix2;

template<typename F>
static F degreeToRadian(const F degree)
{
  return degree * M_PI / 180;
}

template<typename F>
static F radianToDegree(const F radian)
{
  return radian * 180 / M_PI;
}

inline double distance(const Point2 &p1, const Point2 &p2)
{
  Vector2 p3(p2.x()-p1.x(), p2.y()-p1.y());
  return sqrt (p3.squaredNorm());
}

inline double distance(const Point3 &p1, const Point3 &p2)
{
  Vector3 p3(p2.x()-p1.x(), p2.y()-p1.y(), p2.z()-p1.z());
  return sqrt(p3.squaredNorm());
}

inline double distance(const Point4 &p1, const Point4 &p2)
{
  Vector4 p3(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z(), p2.w() - p1.w());
  return std::sqrt(p3.squaredNorm());
}

inline double distance(const double &x1, const double &y1, const double &x2, const double &y2)
{
  Point2 p(x1, y1), q(x2, y2);
  return distance (p, q);
}

inline Vector2 perpendicular(const Vector2 &v)
{
  return Vector2(-v.y(), v.x());
}

inline void rotate(Vector2 &v, const float angleInDegree)
{
  Matrix2 rot;
  double c = std::cos(degreeToRadian(angleInDegree)),
    s = std::sin(degreeToRadian(angleInDegree));
  rot(0, 0) = c;
  rot(0, 1) = -s;
  rot(1, 0) = s;
  rot(1, 1) = c;
  v = rot * v;
}

#endif  // LIBVECTORMAP_MATH_H
