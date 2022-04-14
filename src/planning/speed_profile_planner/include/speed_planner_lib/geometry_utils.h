/**
 * @file geometry_utils.h
 * @brief Collection of geometry computation functions.
 **/

#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include "speed_planner_lib/constants.h"
#include "speed_planner_lib/point2d.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <math.h>
#include <vector>

namespace planning {

double distance2D(const Point2D &p1, const Point2D &p2);

double distance2D(double x1, double y1, double x2, double y2);

const Point2D rotate(const Point2D &in, const double theta);

const Point2D globalToLocal(const Point2D &center, const double theta,
                            const Point2D &p);

const Point2D localToGlobal(const Point2D &center, const double theta,
                            const Point2D &p);

double dotProduct(const Point2D &a, const Point2D &b);

Point2D getIntersectionPointWithCurve(const Point2D &line_start,
                                      const Point2D &line_end,
                                      std::vector<Point2D> ref_path);

Point2D getProjectedPointOnLine(const Point2D &line_start,
                                const Point2D &line_end, const Point2D &p);

Point2D getProjectedPointOnLine(const Point2D &line_start,
                                const Point2D &line_end, const Point2D &p1,
                                const Point2D &p2);

double distanceToLine(const Point2D start, const Point2D end,
                      const Point2D point);

int closestPoint(std::vector<Point2D> ref_path, double x, double y);

bool isOnPath(std::vector<Point2D> path, Point2D check_p, double thresh);

} // namespace planning

#endif // GEOMETRY_UTILS_H
