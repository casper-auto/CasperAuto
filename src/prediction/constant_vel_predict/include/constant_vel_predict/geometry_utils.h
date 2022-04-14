/***************************************************************************************/
/**
\author	    Malcolm Ma
\date	    Feb 1,2017
\version	v1.0.0

\brief


\details

\todo
Unit test needed.

\note
Since Point3D could be constructed with Point2D, theoretically you only need to
create
a function for Point3D if you don't care about the z value, as z will be set to
zero.
For performance improvement it's better to define both version but then you have
to
maintain two copies of similar codes.

*/
/***************************************************************************************/
#ifndef __Honda_Geometry_Utils__
#define __Honda_Geometry_Utils__
#include <math.h>
#include <constant_vel_predict/point.h>

#include <cassert>
#include <cmath>
#include <limits>

namespace geometry {

// if D>0,p is on left side of the line
// if D<0,p is on right side of the line
// if D=0,p is on the line
template <class PointType1, class PointType2, class PointType3>
int64_t judgeDirection(const PointType1 &p1, const PointType2 &p2,
                       const PointType3 &p);

template <class PointType1, class PointType2, class PointType3>
double area(const PointType1 &a, const PointType2 &b, const PointType3 &c);

template <class PointType1, class PointType2, class PointType3>
const double curvature(const PointType1 &p1, const PointType2 &p2,
                       const PointType3 &p);

template <class PointType>
const PointType rotate(const PointType &in, const double theta);

template <class PointType>
const PointType rotate(const PointType &in, const Angle3D &angle);

template <class PointType1, class PointType2>
const PointType2 globalToLocal(const PointType1 &center, const double theta,
                               const PointType2 &p);

template <class PointType1, class PointType2>
const PointType2 globalToLocal(const PointType1 &center, const Angle3D &angle,
                               const PointType2 &p);

template <class PointType1, class PointType2>
const PointType2 localToGlobal(const PointType1 &center, const double theta,
                               const PointType2 &p);

template <class PointType1, class PointType2>
const PointType2 localToGlobal(const PointType1 &center, const Angle3D &angle,
                               const PointType2 &p);

template <class PointType1, class PointType2, class PointType3>
const double distanceToLine(const PointType1 &p1, const PointType2 &p2,
                            const PointType3 &p);

template <class PointType1, class PointType2, class VectorTypeOut>
const VectorTypeOut directionVectorNormalized2D(const PointType1 &start_point,
                                                const PointType2 &end_point);

template <class PointType1, class PointType2, class VectorTypeOut>
const VectorTypeOut directionVectorNormalized3D(const PointType1 &start_point,
                                                const PointType2 &end_point);

double module(const double &x, const double &y);

template <class PointType1, class PointType2>
double distance2D(const PointType1 &p1, const PointType2 &p2);

template <class PointType1>
int findClosestIndex2D(std::vector<PointType1> vect_pts,
                       PointType1 point_input);

// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines
// intersect the intersection point may be stored in the point intersect.
template <class PointType1, class PointType2, class PointType3>
bool lineIntersection2D(const PointType1 &p1_start, const PointType1 &p1_end,
                        const PointType2 &p2_start, const PointType2 &p2_end,
                        PointType3 &intersect);

// Generate a linearly spaced line, start and end included
template <class PointType>
std::vector<PointType> linspace2D(const PointType &start, const PointType &end,
                                  const size_t &steps);

template <class PointType>
std::vector<PointType> ParallelizePath(const std::vector<PointType> &in_path,
                                       const double delta,
                                       const double max_dist = 0.5,
                                       const double min_dist = 1e-3);

/**
Calculates the area of the parallelogram of the three points.
This is actually the same as the area of the triangle defined by the three
points,
multiplied by 2.
@return 2 * triangleArea(a,b,c)
*/
template <class PointType>
double perpDotProduct(const PointType &a, const PointType &b,
                      const PointType &c);

template <class PointType>
double dotProduct(const PointType &a, const PointType &b);

template <class PointType1, class PointType2>
bool isProjectedPointOnLine(const PointType1 &line_start,
                            const PointType1 &line_end, const PointType2 &p);

template <class PointType1, class PointType2>
PointType2 getProjectedPointOnLine(const PointType1 &line_start,
                                   const PointType1 &line_end,
                                   const PointType2 &p);

// ---------------------------- 2D Point functions ----------------------------

double distance(const Point2D &p1, const Point2D &p2);

double distance3D(const Point2D &p1, const Point2D &p2);

double distance(const PolyLine2D &path, const int start = 0,
                const int end = std::numeric_limits<int>::max());

// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines
// intersect the intersection point may be stored in the point intersect.
bool lineIntersection(const Point2D &p1_start, const Point2D &p1_end,
                      const Point2D &p2_start, const Point2D &p2_end,
                      Point2D &intersect);

double distanceToLine(const Point2D start, const Point2D end,
                      const Point2D point);

bool isPointInRect(const Rect r1, const Point2D p);

// ---------------------------- 3D Point functions ----------------------------
double distance(const Point3D &p1, const Point3D &p2);

double distance3D(const Point3D &p1, const Point3D &p2);

/**
Calculates the area of the parallelogram of the three points.
This is actually the same as the area of the triangle defined by the three
points,
multiplied by 2.
@return 2 * triangleArea(a,b,c)
*/
double perpDotProduct(const Point3D &a, const Point3D &b, const Point3D &c);

double dotProduct(const Point3D &a, const Point3D &b);

// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines
// intersect the intersection point may be stored in the point intersect.
bool lineIntersection(const Point3D &p1_start, const Point3D &p1_end,
                      const Point3D &p2_start, const Point3D &p2_end,
                      Point3D &intersect);

bool isInRange(const Point3D &a, const Point3D &b, double range);

bool isInRange2D(const Point3D &a, const Point3D &b, double range);

bool isSamePoint(const Point3D &a, const Point3D &b, double tolerence = 0.1);

bool isSamePoint2D(const Point3D &a, const Point3D &b, double tolerence = 0.1);

// Calculate the length of a path,if the start and end are not provided, the
// function
// calculates the whole path.Start is inclusive and end is exclusive.
double distance(const PolyLine3D &path, const int start = 0,
                const int end = std::numeric_limits<int>::max());

// Calculate the length of a path,if the start and end are not provided, the
// function
// calculates the whole path.Start is inclusive and end is exclusive.
double distance2D(const PolyLine3D &path, const int start = 0,
                  const int end = std::numeric_limits<int>::max());

template <class PointType1, class PointType2, class PointType3>
int64_t judgeDirection(const PointType1 &p1, const PointType2 &p2,
                       const PointType3 &p) {
  double A, B, C, D;
  A = -(p2.y - p1.y);
  B = p2.x - p1.x;
  C = -(A * p1.x + B * p1.y);
  D = A * p.x + B * p.y + C;

  if (D <= 0.00001 && D >= -0.00001) return 0;
  if (D > 0) return 1;
  if (D < 0) return -1;
}

template <class PointType1, class PointType2, class PointType3>
double area(const PointType1 &a, const PointType2 &b, const PointType3 &c) {
  double area_ = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  return area_;
}

template <class PointType1, class PointType2, class PointType3>
const double curvature(const PointType1 &a, const PointType2 &b,
                       const PointType3 &c) {
  double dist1 = distance2D(a, b);
  double dist2 = distance2D(b, c);
  double dist3 = distance2D(c, a);
  double curvature_ = 4 * area(a, b, c) / (dist1 * dist2 * dist3);
  return curvature_;
}

template <class PointType>
const PointType rotate(const PointType &in, const double theta) {
  PointType out;
  double s = sin(theta);
  double c = cos(theta);

  out.x = in.x * c - in.y * s;
  out.y = in.x * s + in.y * c;

  return out;
}

template <class PointType>
const PointType rotate(const PointType &in, const Angle3D &angle) {
  PointType out;
  double s_a = sin(angle.yaw);
  double c_a = cos(angle.yaw);
  double s_c = sin(angle.roll);
  double c_c = cos(angle.roll);
  double s_b = sin(angle.pitch);
  double c_b = cos(angle.pitch);
  out.x = (c_a * c_b) * in.x + (c_a * s_b * s_c - s_a * c_c) * in.y +
          (c_a * s_b * c_c + s_a * s_c) * in.z;
  out.y = (s_a * c_b) * in.x + (s_a * s_b * s_c + c_a * c_c) * in.y +
          (s_a * s_b * c_c - c_a * s_c) * in.z;
  out.z = -s_b * in.x + c_b * s_c * in.y + c_b * c_a * in.z;
  return out;
}

template <class PointType1, class PointType2, class PointType3>
const double distanceToLine(const PointType1 &start, const PointType2 &end,
                            const PointType3 &p) {
  double normalLength = hypot(end.x - start.x, end.y - start.y);
  double distance = (double)((p.x - start.x) * (end.y - start.y) -
                             (p.y - start.y) * (end.x - start.x)) /
                    normalLength;
  return fabs(distance);
}

template <class PointType1, class PointType2>
const PointType2 globalToLocal(const PointType1 &center, const double theta,
                               const PointType2 &p) {
  // Stupid ROS message doesn't support this constructor so I have to assign x y
  // separately...
  // PointType delta(p - center);
  PointType2 delta;
  delta.x = p.x - center.x;
  delta.y = p.y - center.y;
  return rotate(delta, -theta);
}

template <class PointType1, class PointType2>
const PointType2 globalToLocal(const PointType1 &center, const Angle3D &angle,
                               const PointType2 &p) {
  PointType2 delta;
  delta.x = p.x - center.x;
  delta.y = p.y - center.y;
  delta.z = p.z - center.z;
  return rotate(delta, {-angle.roll, -angle.pitch, -angle.yaw});
}

template <class PointType1, class PointType2>
const PointType2 localToGlobal(const PointType1 &center, const double theta,
                               const PointType2 &p) {
  PointType2 out = rotate(p, theta);
  //  out += center;
  out.x += center.x;
  out.y += center.y;
  return out;
}

template <class PointType1, class PointType2>
const PointType2 localToGlobal(const PointType1 &center, const Angle3D &angle,
                               const PointType2 &p) {
  PointType2 out = rotate(p, angle);
  out.x += center.x;
  out.y += center.y;
  out.z += center.z;
  return out;
}

template <class PointType1, class PointType2, class VectorTypeOut>
const VectorTypeOut directionVectorNormalized2D(const PointType1 &start_point,
                                                const PointType2 &end_point) {
  VectorTypeOut dir_vect;
  dir_vect.x = end_point.x - start_point.x;
  dir_vect.y = end_point.y - start_point.y;

  // Normalize
  double norm_vec = hypot(dir_vect.x, dir_vect.y);
  dir_vect.x /= norm_vec;
  dir_vect.y /= norm_vec;

  return dir_vect;
}

template <class PointType1, class PointType2, class VectorTypeOut>
const VectorTypeOut directionVectorNormalized3D(const PointType1 &start_point,
                                                const PointType2 &end_point) {
  VectorTypeOut dir_vect;
  dir_vect.x = end_point.x - start_point.x;
  dir_vect.y = end_point.y - start_point.y;
  dir_vect.z = end_point.z - start_point.z;

  // Normalize
  double norm_vec = hypot(dir_vect.x, dir_vect.y, dir_vect.z);
  dir_vect.x /= norm_vec;
  dir_vect.y /= norm_vec;
  dir_vect.z /= norm_vec;

  return dir_vect;
}

template <class PointType1, class PointType2>
double distance2D(const PointType1 &p1, const PointType2 &p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines
// intersect the intersection point may be stored in the point intersect.
template <class PointType1, class PointType2, class PointType3>
bool lineIntersection2D(const PointType1 &p1_start, const PointType1 &p1_end,
                        const PointType2 &p2_start, const PointType2 &p2_end,
                        PointType3 &intersect) {
  const double &p0_x = p1_start.x;
  const double &p0_y = p1_start.y;
  const double &p1_x = p1_end.x;
  const double &p1_y = p1_end.y;

  const double &p2_x = p2_start.x;
  const double &p2_y = p2_start.y;
  const double &p3_x = p2_end.x;
  const double &p3_y = p2_end.y;

  double s1_x, s1_y, s2_x, s2_y;
  s1_x = p1_x - p0_x;
  s1_y = p1_y - p0_y;
  s2_x = p3_x - p2_x;
  s2_y = p3_y - p2_y;

  double s, t;
  double den = (-s2_x * s1_y + s1_x * s2_y);

  if (std::abs(den) <= 1e-09) {
    // Parallel-Collinear case
    double dist = std::abs(distanceToLine(p1_start, p1_end, p2_start));
    if (dist < 1e-09) {
      // Colinear - Need to check for overlap
      geometry::Point2D pa(0.0, 0.0), pb, pc, pd;
      double theta = atan2(p1_end.y - p1_end.y, p1_end.x - p1_end.x);
      pb = globalToLocal(p1_start, theta, p1_end);    // B
      pc = globalToLocal(p1_start, theta, p2_start);  // C
      pd = globalToLocal(p1_start, theta, p2_end);    // D

      geometry::Point2D p_swap;
      if (pa.x >= pb.x) {
        p_swap = pb;
        pb = pa;
        pa = p_swap;
      }
      if (pc.x >= pd.x) {
        p_swap = pd;
        pd = pc;
        pc = p_swap;
      }

      // Overlap
      if (pb.x - pc.x >= 0 && pd.x - pa.x >= 0) {
        geometry::Point2D p_int(0.0, 0.0);
        // Use the mid point in the overlap interval
        p_int.x = (std::max(pa.x, pc.x) + std::min(pb.x, pd.x)) / 2.0;
        intersect = localToGlobal(p1_start, theta, p_int);
        return true;
      } else {
        return false;
      }
    } else {
      // Parallel
      return false;
    }
  } else {
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / den;
    t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / den;

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
      // Collision detected
      intersect.x = p0_x + (t * s1_x);
      intersect.y = p0_y + (t * s1_y);
      return true;
    }

    return false;  // No collision
  }
}

// Generate a linearly spaced line, start and end included
template <class PointType>
std::vector<PointType> linspace2D(const PointType &start, const PointType &end,
                                  const size_t &steps) {
  if (steps == 0) return {};
  if (steps == 1) return {start};
  if (steps == 2) return {start, end};

  double dx = (end.x - start.x) / (steps - 1);
  double dy = (end.y - start.y) / (steps - 1);

  std::vector<PointType> result(steps);
  PointType p;
  p.x = start.x;
  p.y = start.y;
  for (int i = 0; i < steps; i++) {
    result[i] = (p);
    p.x += dx;
    p.y += dy;
  }
  return result;
}

template <class PointType>
std::vector<PointType> ParallelizePath(const std::vector<PointType> &in_path,
                                       const double delta,
                                       const double max_dist,
                                       const double min_dist) {
  if (in_path.size() < 2) return {};
  std::vector<PointType> in_path_{in_path.front()};
  in_path_.reserve(in_path.size());
  for (int i = 1; i < in_path.size(); i++) {
    if (std::hypot(in_path[i].x - in_path[i - 1].x,
                   in_path[i].y - in_path[i - 1].y) > min_dist)
      in_path_.push_back(in_path[i]);
  }

  // Calculate the heading of each line segment for transform
  std::vector<double> theta(in_path_.size());
  for (int i = 0; i + 1 < in_path_.size(); i++) {
    theta[i] = atan2(in_path_[i + 1].y - in_path_[i].y,
                     in_path_[i + 1].x - in_path_[i].x);
  }
  theta.back() = theta[theta.size() - 2];

  // Calculate offset line for each segment
  typedef std::pair<PointType, PointType> LineType;
  std::vector<LineType> segments(in_path_.size() - 1);
  PointType delta_point;
  delta_point.x = 0;
  delta_point.y = delta;
  for (int i = 0; i + 1 < in_path_.size(); i++) {
    auto start = localToGlobal(in_path_[i], theta[i], delta_point);
    auto end = localToGlobal(in_path_[i + 1], theta[i + 1], delta_point);
    segments[i] = LineType{start, end};
  }

  // Handle the joint of two segments, this includes:
  //    Eliminate the points are that too close
  //    Trim the collision part
  //    Linspace the point that are too far away
  // In the following process, it always assumes that the start of previous
  // point is handled and it checks the end of the previous segment and the
  // start of the current segment
  std::vector<PointType> out_path;
  out_path.reserve(in_path_.size());
  out_path.push_back(segments.front().first);
  for (int i = 1; i < segments.size(); i++) {
    PointType intersect_point;
    if (lineIntersection2D(segments[i].first, segments[i].second,
                           segments[i - 1].first, segments[i - 1].second,
                           intersect_point)) {
      out_path.push_back(intersect_point);
      continue;
    }

    auto dist = std::hypot(segments[i - 1].second.x - segments[i].second.x,
                           segments[i - 1].first.y - segments[i].first.y);
    if (dist < min_dist) {
      out_path.push_back(segments[i - 1].second);
      continue;
    } else if (dist > max_dist) {
      // Linspace
      size_t steps = dist / max_dist + 1;
      auto new_points =
          linspace2D(segments[i - 1].second, segments[i].first, steps);
      out_path.insert(out_path.end(), new_points.begin(), new_points.end());
    }
  }
  out_path.push_back(segments.back().second);
  return out_path;
}

template <class PointType>
double perpDotProduct(const PointType &a, const PointType &b,
                      const PointType &c) {
  return (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x);
}

template <class PointType>
double dotProduct(const PointType &a, const PointType &b) {
  return a.x * b.x + a.y * b.y;
}

template <class PointType1, class PointType2>
bool isProjectedPointOnLine(const PointType1 &line_start,
                            const PointType1 &line_end, const PointType2 &p) {
  PointType2 e1;
  e1.x = line_end.x - line_start.x;
  e1.y = line_end.y - line_start.y;
  double recArea = dotProduct(e1, e1);
  PointType2 e2;
  e2.x = p.x - line_start.x;
  e2.y = p.y - line_start.y;
  double val = dotProduct(e1, e2);
  return (val >= 0 && val <= recArea);
}

template <class PointType1, class PointType2>
PointType2 getProjectedPointOnLine(const PointType1 &line_start,
                                   const PointType1 &line_end,
                                   const PointType2 &p) {
  PointType2 e1;
  e1.x = line_end.x - line_start.x;
  e1.y = line_end.y - line_start.y;
  PointType2 e2;
  e2.x = p.x - line_start.x;
  e2.y = p.y - line_start.y;
  double val = dotProduct(e1, e2);

  // get squared length of e1
  double len2 = e1.x * e1.x + e1.y * e1.y;
  PointType2 projected;
  projected.x = line_start.x + (val * e1.x) / len2;
  projected.y = line_start.y + (val * e1.y) / len2;
  return projected;
}

inline bool doubleSame(double a, double b, double tolerance) {
  return std::fabs(a - b) < tolerance;
}

inline bool floatSame(float a, float b, float tolerance) {
  return std::fabs(a - b) < tolerance;
}

template <class PointType1>
int findClosestIndex2D(std::vector<PointType1> vect_pts,
                       PointType1 point_input) {
  double min_distance = std::numeric_limits<double>::max();
  int ind_closest = -1;
  for (int i = 0; i < vect_pts.size(); i++) {
    double distance_val = distance2D(point_input, vect_pts[i]);
    if (distance_val < min_distance) {
      min_distance = distance_val;
      ind_closest = i;
    }
  }
  return ind_closest;
}

}  // end of namespace geometry
#endif
