/**
 * @file frenet_frame.h
 * @brief Frenet class definition.
 **/

#ifndef FRENET_FRAME_H
#define FRENET_FRAME_H

#include <iostream>
#include <limits>
#include <math.h>
#include <stdio.h>
#include <string>
#include <tgmath.h>
#include <vector>

#include "speed_planner_lib/geometry_utils.h"
#include "speed_planner_lib/log.h"
#include "speed_planner_lib/point2d.h"
#include "speed_planner_lib/point_frenet.h"

namespace planning {

/**
 * @class Frenet
 * @brief Frenet operates all the representation and convertion in Frenet Frame.
 **/

class Frenet {
public:
  // Constructor
  Frenet();
  Frenet(const std::vector<Point2D> &path);

  // Convert back and forth between Frenet and Cartesian coordinates
  void ToFrenet(const Point2D p_xy, const int closest, Point_Frenet &p_sd);
  void ToFrenet(const Point2D p_xy, const int closest, Point_Frenet &p_sd,
                double &road_dir);
  void ToCartesian(const Point_Frenet p_sd, Point2D &p_xy);
  void SetPath(const std::vector<Point2D> &path);

private:
  // Waypoints of the center line of the road in the global map coordinates
  std::vector<Point2D> path_m;
  std::vector<double> path_s_m;
  void CreateSPath();
};

} // namespace planning

#endif // FRENET_FRAME_H
