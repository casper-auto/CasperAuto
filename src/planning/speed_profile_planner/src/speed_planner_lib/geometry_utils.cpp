#include "speed_planner_lib/geometry_utils.h"

namespace planning {

double distance2D(const Point2D &p1, const Point2D &p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

double distance2D(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return sqrt(dx * dx + dy * dy);
}

const Point2D rotate(const Point2D &in, const double theta) {
  Point2D out;
  double s = sin(theta);
  double c = cos(theta);

  out.x = in.x * c - in.y * s;
  out.y = in.x * s + in.y * c;

  return out;
}

const Point2D globalToLocal(const Point2D &center, const double theta,
                            const Point2D &p) {
  Point2D delta;
  delta.x = p.x - center.x;
  delta.y = p.y - center.y;
  return rotate(delta, -theta);
}

const Point2D localToGlobal(const Point2D &center, const double theta,
                            const Point2D &p) {
  Point2D out = rotate(p, theta);
  out.x += center.x;
  out.y += center.y;
  return out;
}

double dotProduct(const Point2D &a, const Point2D &b) {
  return a.x * b.x + a.y * b.y;
}

Point2D getIntersectionPointWithCurve(const Point2D &line_start,
                                      const Point2D &line_end,
                                      std::vector<Point2D> ref_path) {
  int n = 100;
  double min_dist = std::numeric_limits<double>::max();
  Point2D intersection = line_start;
  double distance = line_start.DistanceTo(line_end);
  for (int i = 0; i <= n; i++) {
    double cur_x = line_start.x + (line_end.x - line_start.x) / n * i;
    double cur_y = line_start.y + (line_end.y - line_start.y) / n * i;
    Point2D cur_p(cur_x, cur_y);
    int idx = closestPoint(ref_path, cur_x, cur_y);
    if (min_dist > cur_p.DistanceTo(ref_path[idx])) {
      min_dist = cur_p.DistanceTo(ref_path[idx]);
      intersection = cur_p;
    }
  }
  return intersection;
}

Point2D getProjectedPointOnLine(const Point2D &line_start,
                                const Point2D &line_end, const Point2D &p) {
  // get dot product of e1, e2
  Point2D e1(line_end.x - line_start.x, line_end.y - line_start.y);
  Point2D e2(p.x - line_start.x, p.y - line_start.y);
  double val = dotProduct(e1, e2);
  double len2 = e1.x * e1.x + e1.y * e1.y;
  return Point2D((line_start.x + (val * e1.x) / len2),
                 (line_start.y + (val * e1.y) / len2));
}

double distanceToLine(const Point2D start, const Point2D end,
                      const Point2D point) {
  double normalLength = std::hypot(end.x - start.x, end.y - start.y);
  double distance = (double)((point.x - start.x) * (end.y - start.y) -
                             (point.y - start.y) * (end.x - start.x)) /
                    normalLength;
  return fabs(distance);
}

int closestPoint(std::vector<Point2D> ref_path, double x, double y) {
  int index = 0;
  int closest_index = 0;
  double min_dist = kDBL_MAX;
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

bool isOnPath(std::vector<Point2D> path, Point2D check_p, double thresh) {
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

} // namespace planning
