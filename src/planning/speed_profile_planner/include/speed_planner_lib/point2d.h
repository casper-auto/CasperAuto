/**
 * @file
 * @brief A class of 2D point.
 */

#ifndef POINT2D_H
#define POINT2D_H

#include <cmath> // std::hypot square root since c++ 11

namespace planning {

/**
 * @struct Point2D
 *
 * @brief Implementation of 2D point.
 */
struct Point2D {
  Point2D() : x(0), y(0) {}

  Point2D(double _x, double _y) : x(_x), y(_y) {}

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

  double DistanceTo(const Point2D &other) const {
    return std::hypot(x - other.x, y - other.y);
  }

  double x, y;
};

} // namespace planning

#endif // POINT2D_H
