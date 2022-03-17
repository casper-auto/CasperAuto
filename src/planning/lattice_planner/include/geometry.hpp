/**********************************************
 *  Created on: Aug. 24, 2021
 *      Author: Peng Xu
 **********************************************/
#pragma once

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <array>
#include <vector>
#include <limits>
#include <cfloat>

namespace geometry {

struct Point2D {
  Point2D() : x(0), y(0) {}

  Point2D(double x, double y) : x(x), y(y) {}

  // Point2D(const Point3D& point):x(point.x),y(point.y){}

  Point2D &operator+=(const Point2D &rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
  }

  Point2D &operator-=(const Point2D &rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
  }

  friend Point2D operator+(Point2D lhs, const Point2D &rhs) {
    lhs += rhs;
    return lhs;
  }

  friend Point2D operator-(Point2D lhs, const Point2D &rhs) {
    lhs -= rhs;
    return lhs;
  }

  friend Point2D operator*(Point2D lhs, const int scale) {
    lhs.x *= scale;
    lhs.y *= scale;
    return lhs;
  }

  friend Point2D operator*(Point2D lhs, const double scale) {
    lhs.x *= scale;
    lhs.y *= scale;
    return lhs;
  }

  friend Point2D operator/(Point2D lhs, const double scale) {
    lhs.x /= scale;
    lhs.y /= scale;
    return lhs;
  }

  double x, y;
};

struct Point3D {
  Point3D() : x(0), y(0), z(0) {}

  Point3D(const Point2D &point) : x(point.x), y(point.y), z(0) {}

  Point3D(const double x, const double y, const double z) : x(x), y(y), z(z) {}

  Point3D &operator+=(const Point3D &rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }

  Point3D &operator-=(const Point3D &rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z += rhs.z;
    return *this;
  }

  friend Point3D operator+(Point3D lhs, const Point3D &rhs) {
    lhs += rhs;
    return lhs;
  }

  friend Point3D operator-(Point3D lhs, const Point3D &rhs) {
    lhs -= rhs;
    return lhs;
  }

  bool operator<(const Point3D &rhs) const {
    return x < rhs.x || (x == rhs.x && (y < rhs.y)) ||
           (x == rhs.x && y == rhs.y && z < rhs.z);
  }

  double x, y, z;
};

struct Pose2D {
  Point2D position;
  double yaw;

  Pose2D() {
    this->position = Point2D();
    this->yaw = 0;
  }

  Pose2D(Point2D position, double yaw) {
    this->position = position;
    this->yaw = yaw;
  }

  Pose2D(double x, double y, double yaw) {
    this->position = Point2D(x, y);
    this->yaw = yaw;
  }
};

struct Line2D {
  Line2D() : start(), end() {}

  Line2D(const Point2D &start, const Point2D &end) : start(start), end(end) {}

  Point2D start;
  Point2D end;
};

typedef std::vector<Point2D> PolyLine2D;

typedef std::vector<Point3D> PolyLine3D;

struct Rect {
  std::array<Point2D, 4> p;
};

struct Angle3D {
  double roll;
  double pitch;
  double yaw;
};

struct Point_Frenet {
  Point_Frenet() : s(0.0), d(0.0) {}

  Point_Frenet(double s, double d) : s(s), d(d) {}

  Point_Frenet(const Point_Frenet &p) : s(p.s), d(p.d) {}

  double s, d;
};

inline double distance2D(const Point2D &p1, const Point2D &p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

inline double distance2D(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return sqrt(dx * dx + dy * dy);
}

inline const Point2D rotate(const Point2D &in, const double theta) {
  Point2D out;
  double s = sin(theta);
  double c = cos(theta);

  out.x = in.x * c - in.y * s;
  out.y = in.x * s + in.y * c;

  return out;
}

inline const Point2D globalToLocal(const Point2D &center, const double theta,
                            const Point2D &p) {
  Point2D delta;
  delta.x = p.x - center.x;
  delta.y = p.y - center.y;
  return rotate(delta, -theta);
}

inline const Point2D localToGlobal(const Point2D &center, const double theta,
                            const Point2D &p) {
  Point2D out = rotate(p, theta);
  out.x += center.x;
  out.y += center.y;
  return out;
}

inline double dotProduct(const Point2D &a, const Point2D &b) {
  return a.x * b.x + a.y * b.y;
}

inline Point2D getProjectedPointOnLine(const Point2D &line_start,
                                const Point2D &line_end, const Point2D &p) {
  // get dot product of e1, e2
  Point2D e1(line_end.x - line_start.x, line_end.y - line_start.y);
  Point2D e2(p.x - line_start.x, p.y - line_start.y);
  double val = dotProduct(e1, e2);
  double len2 = e1.x * e1.x + e1.y * e1.y;
  return Point2D((line_start.x + (val * e1.x) / len2),
                 (line_start.y + (val * e1.y) / len2));
}

inline double distanceToLine(const Point2D start, const Point2D end,
                      const Point2D point) {
  double normalLength = std::hypot(end.x - start.x, end.y - start.y);
  double distance = (double)((point.x - start.x) * (end.y - start.y) -
                             (point.y - start.y) * (end.x - start.x)) /
                    normalLength;
  return fabs(distance);
}

inline int closestPoint(std::vector<Point2D> ref_path, double x, double y) {
  int index = 0;
  int closest_index = 0;
  double min_dist = DBL_MAX;
  for (auto p : ref_path) {
    double dist = distance2D(p.x, p.y, x, y);
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = index;
    }
    index++;
  }
  return closest_index;
}

inline int closestPoint(std::vector<Point2D> ref_path, Point2D query) {
  int index = 0;
  int closest_index = 0;
  double min_dist = DBL_MAX;
  for (auto p : ref_path) {
    double dist = distance2D(p.x, p.y, query.x, query.y);
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = index;
    }
    index++;
  }
  return closest_index;
}

inline Point2D getIntersectionPointWithCurve(const Point2D &line_start,
                                      const Point2D &line_end,
                                      std::vector<Point2D> ref_path) {
  int n = 100;
  double min_dist = std::numeric_limits<double>::max();
  Point2D intersection = line_start;
  double distance = distance2D(line_start, line_end);
  for (int i = 0; i <= n; i++) {
    double cur_x = line_start.x + (line_end.x - line_start.x) / n * i;
    double cur_y = line_start.y + (line_end.y - line_start.y) / n * i;
    Point2D cur_p(cur_x, cur_y);
    int idx = closestPoint(ref_path, cur_x, cur_y);
    if (min_dist > distance2D(cur_p, ref_path[idx])) {
      min_dist = distance2D(cur_p, ref_path[idx]);
      intersection = cur_p;
    }
  }
  return intersection;
}

inline bool isOnPath(std::vector<Point2D> path, Point2D check_p, double thresh) {
  int closest_idx = closestPoint(path, check_p.x, check_p.y);
  Point2D pa, pb;
  if (closest_idx == path.size() - 1) {
    pa = path[closest_idx - 1];
    pb = path[closest_idx];
  } else {
    pa = path[closest_idx];
    pb = path[closest_idx + 1];
  }
  double dist = distanceToLine(pa, pb, check_p);
  return dist < thresh;
}

inline bool isOnPath(std::vector<Pose2D> path, Point2D check_p, double thresh) {
  std::vector<Point2D> points(path.size());
  for (int i = 0; i < path.size(); i++) {
    points[i] = path[i].position;
  }
  return isOnPath(points, check_p, thresh);
}

} // namespace geometry

#endif // GEOMETRY_H
