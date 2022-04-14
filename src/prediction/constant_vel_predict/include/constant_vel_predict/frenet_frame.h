#ifndef FRENET_H
#define FRENET_H

#include <math.h>
#include <constant_vel_predict/geometry_utils.h>
#include <constant_vel_predict/point.h>
#include <stdio.h>
#include <tgmath.h>

#include <iostream>
#include <string>
#include <vector>

// TO DO: Maybe make the path a pointer for memory saving

namespace geometry {

class Frenet {
 public:
  // Constructor
  Frenet();
  Frenet(const std::vector<Point2D> &path, bool spec_origin = false,
         Point2D origin = Point2D());

  // Convert back and forth between Frenet and Cartesian coordinates
  bool ToFrenet(const Point2D p_xy, Point_Frenet &p_sd, int closest = -1) const;
  bool ToFrenet(const Point2D p_xy, Point_Frenet &p_sd, double &road_dir,
                int closest = -1) const;
  bool ToFrenetDirectional(const Point2D p_xy, const double theta,
                           Point_Frenet &p_sd) const;
  bool ToFrenetDirectional(const Point2D p_xy, const double theta,
                           Point_Frenet &p_sd, double &road_dir) const;
  bool ToCartesian(const Point_Frenet p_sd, Point2D &p_xy) const;
  bool ToCartesian(const Point_Frenet p_sd, Point2D &p_xy,
                   double &road_dir) const;
  void SetPath(const std::vector<Point2D> &path, bool spec_origin = false,
               Point2D origin = Point2D());
  std::vector<Point2D> GetPath() const { return path_m; }
  std::vector<double> GetSPath() const { return path_s_m; }
  bool PathValid() const { return !path_m.empty() && ~path_s_m.empty(); }
  int findPrevSPath(Point_Frenet point_input) const;
  int findClosestIndex2DWithDirection(Point2D point_input, double theta) const;

 private:
  // Waypoints of the center line of the road in the global map coordinates
  std::vector<Point2D> path_m;
  std::vector<double> path_s_m;
  std::vector<double> path_theta_m;
  void CreateSPath(bool spec_origin = false, Point2D origin = Point2D());
};

}  // end of namespace geometry

#endif
