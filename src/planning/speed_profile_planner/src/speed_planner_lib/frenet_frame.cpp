/**
 * @file frenet_frame.cpp
 * @brief Frenet class definition.
 **/

#include "speed_planner_lib/frenet_frame.h"

namespace planning {

Frenet::Frenet() {
  // Copy the path
  path_m = std::vector<Point2D>(0);
}

Frenet::Frenet(const std::vector<Point2D> &path) {
  // Copy the path
  path_m = path;
  CreateSPath();
}

void Frenet::SetPath(const std::vector<Point2D> &path) {
  path_m = path;
  CreateSPath();
}

void Frenet::CreateSPath() {
  if (path_m.empty())
    return;

  // Create the s path based on distance
  path_s_m.resize(path_m.size());
  path_s_m[0] = 0.0;
  for (auto i = 1; i < path_s_m.size(); ++i) {
    path_s_m[i] = distance2D(path_m[i], path_m[i - 1]) + path_s_m[i - 1];
  }
}

void Frenet::ToFrenet(const Point2D p_xy, const int closest,
                      Point_Frenet &p_sd) {
  double dummy;
  ToFrenet(p_xy, closest, p_sd, dummy);
}

// It assumes that we have a closest point index (usually from ANN)
void Frenet::ToFrenet(const Point2D p_xy, const int closest, Point_Frenet &p_sd,
                      double &road_dir) {
  if (path_m.empty() || path_s_m.empty()) {
    DEBUG_MSG("Empty path. Cannot compute Frenet");
    p_sd.s = std::numeric_limits<double>::lowest();
    p_sd.d = std::numeric_limits<double>::lowest();
    return;
  }

  // Determine the indices of the 2 closest points
  Point2D local_p;
  int prev_idx;
  bool use_previous;
  if (closest < path_m.size()) {
    // Check if we are at the end of the segment
    if (closest == path_m.size() - 1) {
      use_previous = true;
    } else if (closest == 0) {
      use_previous = false;
    } else {
      double dist_prev = distance2D(path_m[closest - 1], p_xy);
      double dist_next = distance2D(path_m[closest + 1], p_xy);

      if (dist_prev <= dist_next) {
        use_previous = true;
      } else {
        use_previous = false;
      }
    }

    // Get the 2 points
    Point2D p1, p2;
    if (use_previous) {
      p1.x = path_m[closest - 1].x;
      p1.y = path_m[closest - 1].y;
      p2.x = path_m[closest].x;
      p2.y = path_m[closest].y;
      prev_idx = closest - 1;
    } else {
      p1.x = path_m[closest].x;
      p1.y = path_m[closest].y;
      p2.x = path_m[closest + 1].x;
      p2.y = path_m[closest + 1].y;
      prev_idx = closest;
    }

    // Get the point in the local coordinate with center p1
    double theta = atan2(p2.y - p1.y, p2.x - p1.x);
    road_dir = theta;
    local_p = globalToLocal(p1, theta, p_xy);

    // Get the coordinates in the Frenet frame
    p_sd.s = path_s_m[prev_idx] + local_p.x;
    p_sd.d = local_p.y;

  } else {
    // Wrong indexing from ANN
    DEBUG_MSG("Incorrect index");
    p_sd.s = std::numeric_limits<double>::lowest();
    p_sd.d = std::numeric_limits<double>::lowest();
    road_dir = 100.0; // BAD VALUE
  }
}

void Frenet::ToCartesian(const Point_Frenet p_sd, Point2D &p_xy) {
  if (path_m.empty() || path_s_m.empty()) {
    DEBUG_MSG("Empty path. Cannot compute Frenet");
    p_xy.x = std::numeric_limits<double>::lowest();
    p_xy.y = std::numeric_limits<double>::lowest();
    return;
  }
  int prev_point;

  // If the value is out of the actual path send a warning
  if (p_sd.s < 0.0 || p_sd.s > path_s_m.back()) {
    if (p_sd.s < 0.0) {
      prev_point = 0;
    } else {
      prev_point = path_s_m.size() - 2;
    }
    DEBUG_MSG("Point is out of the S path: p_sd.s = " << p_sd.s);
  } else {
    // Find the previous point
    auto it = std::lower_bound(path_s_m.begin(), path_s_m.end(), p_sd.s);
    prev_point = it - path_s_m.begin() - 1;
  }

  Point2D p1, p2;
  p1.x = path_m[prev_point].x;
  p1.y = path_m[prev_point].y;
  p2.x = path_m[prev_point + 1].x;
  p2.y = path_m[prev_point + 1].y;

  // Transform from local to global
  double theta = atan2(p2.y - p1.y, p2.x - p1.x);
  p_xy =
      localToGlobal(p1, theta, Point2D(p_sd.s - path_s_m[prev_point], p_sd.d));
}

} // namespace planning
