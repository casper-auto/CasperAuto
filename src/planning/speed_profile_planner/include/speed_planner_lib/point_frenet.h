/**
 * @file point_frenet.h
 * @brief Point_Frenet class definition.
 **/

#ifndef POINT_FRENET_H
#define POINT_FRENET_H

namespace planning {

/**
 * @class Point_Frenet
 * @brief A data structure for representing a point in Frenet Frame.
 */
struct Point_Frenet {
  Point_Frenet() : s(0.0), d(0.0) {}

  Point_Frenet(double s, double d) : s(s), d(d) {}

  Point_Frenet(const Point_Frenet &p) : s(p.s), d(p.d) {}

  double s, d;
};

} // namespace planning

#endif // POINT_FRENET_H
