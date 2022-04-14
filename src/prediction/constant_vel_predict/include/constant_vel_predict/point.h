/***************************************************************************************/
/**
\author	    Malcolm Ma
\date	    Feb 1,2017
\version	v1.0.0

\brief


\details

\todo
Unit test needed.
*/
/***************************************************************************************/
#ifndef __Honda_Point__
#define __Honda_Point__
#include <array>
#include <vector>

namespace geometry {

struct Point2D {
  Point2D() : x(0), y(0) {}

  Point2D(double x, double y) : x(x), y(y) {}

  // Point2D(const Point3D& point):x(point.x),y(point.y){}

  Point2D& operator+=(const Point2D& rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
  }

  Point2D& operator-=(const Point2D& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
  }

  friend Point2D operator+(Point2D lhs, const Point2D& rhs) {
    lhs += rhs;
    return lhs;
  }

  friend Point2D operator-(Point2D lhs, const Point2D& rhs) {
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

  Point3D(const Point2D& point) : x(point.x), y(point.y), z(0) {}

  Point3D(const double x, const double y, const double z) : x(x), y(y), z(z) {}

  Point3D& operator+=(const Point3D& rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }

  Point3D& operator-=(const Point3D& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z += rhs.z;
    return *this;
  }

  friend Point3D operator+(Point3D lhs, const Point3D& rhs) {
    lhs += rhs;
    return lhs;
  }

  friend Point3D operator-(Point3D lhs, const Point3D& rhs) {
    lhs -= rhs;
    return lhs;
  }

  bool operator<(const Point3D& rhs) const {
    return x < rhs.x || (x == rhs.x && (y < rhs.y)) ||
           (x == rhs.x && y == rhs.y && z < rhs.z);
  }

  double x, y, z;
};

struct Line2D {
  Line2D() : start(), end() {}

  Line2D(const Point2D& start, const Point2D& end) : start(start), end(end) {}

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

  Point_Frenet(const Point_Frenet& p) : s(p.s), d(p.d) {}

  double s, d;
};

}  // end of namespace geometry

#endif
