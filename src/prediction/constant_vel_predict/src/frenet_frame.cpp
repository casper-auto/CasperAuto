#include <constant_vel_predict/frenet_frame.h>

#include <limits>

namespace geometry {

Frenet::Frenet() {
  // Copy the path
  path_m = std::vector<Point2D>(0);
}

Frenet::Frenet(const std::vector<Point2D> &path, bool spec_origin,
               Point2D origin) {
  // Copy the path
  path_m = path;
  CreateSPath(spec_origin, origin);
}

void Frenet::SetPath(const std::vector<Point2D> &path, bool spec_origin,
                     Point2D origin) {
  path_m = path;
  CreateSPath(spec_origin, origin);
}

void Frenet::CreateSPath(bool spec_origin, Point2D origin) {
  if (path_m.empty()) return;

  path_s_m.resize(path_m.size());
  path_theta_m.resize(path_m.size());

  if (!spec_origin) {
    // Create the s path based on distance
    path_s_m[0] = 0.0;
    for (auto i = 1; i < path_s_m.size(); ++i) {
      path_s_m[i] = distance(path_m[i], path_m[i - 1]) + path_s_m[i - 1];
      Point2D dir_vec = path_m[i] - path_m[i - 1];
      path_theta_m[i - 1] = atan2(dir_vec.y, dir_vec.x);
    }
    path_theta_m[path_theta_m.size() - 1] =
        path_theta_m[path_theta_m.size() - 2];
  } else {
    int ind_closest = findClosestIndex2D(path_m, origin);
    path_s_m[ind_closest] = 0.0;

    for (int i = ind_closest - 1; i >= 0; i--) {
      path_s_m[i] =
          path_s_m[i + 1] - geometry::distance(path_m[i], path_m[i + 1]);
      Point2D dir_vec = path_m[i + 1] - path_m[i];
      path_theta_m[i] = atan2(dir_vec.y, dir_vec.x);
    }

    for (int i = ind_closest + 1; i < path_m.size(); i++) {
      path_s_m[i] =
          path_s_m[i - 1] + geometry::distance(path_m[i], path_m[i - 1]);
      Point2D dir_vec = path_m[i] - path_m[i - 1];
      path_theta_m[i - 1] = atan2(dir_vec.y, dir_vec.x);
    }
    path_theta_m[path_theta_m.size() - 1] =
        path_theta_m[path_theta_m.size() - 2];
  }
}

bool Frenet::ToFrenet(const Point2D p_xy, Point_Frenet &p_sd,
                      int closest) const {
  double dummy;
  return ToFrenet(p_xy, p_sd, dummy, closest);
}

bool Frenet::ToFrenetDirectional(const Point2D p_xy, const double theta,
                                 Point_Frenet &p_sd) const {
  double dummy;
  int closest = findClosestIndex2DWithDirection(p_xy, theta);
  if (closest == -1) {
    std::cout << "Can't find index in frenet directional" << std::endl;
    return false;
  }
  return ToFrenet(p_xy, p_sd, dummy, closest);
}

bool Frenet::ToFrenetDirectional(const Point2D p_xy, const double theta,
                                 Point_Frenet &p_sd, double &road_dir) const {
  int closest = findClosestIndex2DWithDirection(p_xy, theta);
  if (closest == -1) {
    std::cout << "Can't find index in frenet directional" << std::endl;
    return false;
  }
  return ToFrenet(p_xy, p_sd, road_dir, closest);
}

bool Frenet::ToFrenet(const Point2D p_xy, Point_Frenet &p_sd, double &road_dir,
                      int closest) const {
  if (path_m.empty() || path_s_m.empty()) {
    std::cout << "Empty path. Cannot compute Frenet" << std::endl;
    p_sd.s = std::numeric_limits<double>::lowest();
    p_sd.d = std::numeric_limits<double>::lowest();
    return false;
  }

  // Not defined by the user, compute again
  if (closest == -1) {
    closest = findClosestIndex2D(path_m, p_xy);
  }

  uint32_t closest_ind, prev_idx;
  if (closest < 0) {
    std::cout << "Wrong closest index. Cannot compute Frenet" << std::endl;
    p_sd.s = std::numeric_limits<double>::lowest();
    p_sd.d = std::numeric_limits<double>::lowest();
    return false;
  } else {
    closest_ind = uint16_t(closest);
  }

  // Determine the indices of the 2 closest points
  Point2D local_p;
  bool use_previous;
  if (closest_ind < path_m.size()) {
    // Check if we are at the end of the segment
    if (closest_ind == path_m.size() - 1) {
      use_previous = true;
    } else if (closest == 0) {
      use_previous = false;
    } else {
      double dist_prev = distance(path_m[closest_ind - 1], p_xy);
      double dist_next = distance(path_m[closest_ind + 1], p_xy);

      if (dist_prev <= dist_next) {
        use_previous = true;
      } else {
        use_previous = false;
      }
    }

    // Get the 2 points
    Point2D p1, p2;
    if (use_previous) {
      p1.x = path_m[closest_ind - 1].x;
      p1.y = path_m[closest_ind - 1].y;
      p2.x = path_m[closest_ind].x;
      p2.y = path_m[closest_ind].y;
      prev_idx = closest_ind - 1;
    } else {
      p1.x = path_m[closest_ind].x;
      p1.y = path_m[closest_ind].y;
      p2.x = path_m[closest_ind + 1].x;
      p2.y = path_m[closest_ind + 1].y;
      prev_idx = closest_ind;
    }

    // Get the point in the local coordinate with center p1
    road_dir = atan2(p2.y - p1.y, p2.x - p1.x);
    local_p = globalToLocal(p1, road_dir, p_xy);

    // Get the coordinates in the Frenet frame
    p_sd.s = path_s_m[prev_idx] + local_p.x;
    p_sd.d = local_p.y;

  } else {
    // Wrong indexing from ANN
    std::cout << "Incorrect index" << std::endl;
    p_sd.s = std::numeric_limits<double>::lowest();
    p_sd.d = std::numeric_limits<double>::lowest();
    road_dir = 100.0;  // BAD VALUE
    return false;
  }

  return true;
}

bool Frenet::ToCartesian(const Point_Frenet p_sd, Point2D &p_xy) const {
  double dummy;
  return ToCartesian(p_sd, p_xy, dummy);
}

bool Frenet::ToCartesian(const Point_Frenet p_sd, Point2D &p_xy,
                         double &road_dir) const {
  if (path_m.empty() || path_s_m.empty()) {
    std::cout << "Empty path. Cannot compute Cartesian" << std::endl;
    p_xy.x = std::numeric_limits<double>::lowest();
    p_xy.y = std::numeric_limits<double>::lowest();
    return false;
  }
  uint32_t prev_point_ind;

  // If the value is out of the actual path send a warning
  if (p_sd.s <= path_s_m.front() || p_sd.s >= path_s_m.back()) {
    if (p_sd.s < path_s_m.front()) {
      prev_point_ind = 0;
    } else {
      prev_point_ind = uint32_t(path_s_m.size() - 2);
    }
    // std::cout << "Point is out of the S path: p_sd.s = " << p_sd.s <<
    // std::endl;
  } else {
    // Find the previous point
    auto it = std::lower_bound(path_s_m.begin(), path_s_m.end(), p_sd.s);
    prev_point_ind = uint32_t(it - path_s_m.begin() - 1);
  }

  Point2D p1, p2;
  p1.x = path_m[prev_point_ind].x;
  p1.y = path_m[prev_point_ind].y;
  p2.x = path_m[prev_point_ind + 1].x;
  p2.y = path_m[prev_point_ind + 1].y;

  // Transform from local to global
  road_dir = atan2(p2.y - p1.y, p2.x - p1.x);

  p_xy = localToGlobal(p1, road_dir,
                       Point2D(p_sd.s - path_s_m[prev_point_ind], p_sd.d));
  return true;
}

int Frenet::findPrevSPath(Point_Frenet point_input) const {
  if (path_s_m.empty()) {
    std::cout << "Empty S path. Cannot compute closest" << std::endl;
    return -1;
  }
  int prev_point_ind;

  // If the value is out of the actual path send a warning
  if (point_input.s <= path_s_m.front() || point_input.s >= path_s_m.back()) {
    if (point_input.s <= path_s_m.front()) {
      prev_point_ind = 0;
    } else {
      prev_point_ind = int(path_s_m.size() - 2);
    }
  } else {
    // Find the previous point
    auto it = std::lower_bound(path_s_m.begin(), path_s_m.end(), point_input.s);
    prev_point_ind = int(it - path_s_m.begin() - 1);
  }
  return prev_point_ind;
}

int Frenet::findClosestIndex2DWithDirection(Point2D point_input,
                                            double theta) const {
  double min_distance = std::numeric_limits<double>::max();
  int ind_closest = -1;
  for (int i = 0; i < path_m.size(); i++) {
    double aux_angle =
        atan2(sin(path_theta_m[i] - theta), cos(path_theta_m[i] - theta));
    if (aux_angle < -3 * M_PI / 4 || aux_angle > 3 * M_PI / 4) {
      continue;
    }

    double distance_val = distance2D(point_input, path_m[i]);
    if (distance_val < min_distance) {
      min_distance = distance_val;
      ind_closest = i;
    }
  }
  return ind_closest;
}

}  // end of namespace geometry
