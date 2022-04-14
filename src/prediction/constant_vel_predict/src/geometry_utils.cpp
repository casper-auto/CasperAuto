#include <math.h>
#include <constant_vel_predict/geometry_utils.h>

namespace geometry {
// ---------------------------- 2D Point functions ----------------------------

double module(const double &x, const double &y) { return sqrt(x * x + y * y); }

double distance(const Point2D &p1, const Point2D &p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

double distance3D(const Point2D &p1, const Point2D &p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

double distance(const PolyLine2D &path, const int start, const int end) {
  if (path.size() < 2 || start < 0 || start + 1 >= end || start >= path.size())
    return 0;

  double dist = 0;
  for (int i = start; i + 1 < path.size() && i + 1 < end; i++)
    dist += distance(path[i], path[i + 1]);
  return dist;
}

double perpDotProduct(const Point2D &a, const Point2D &b, const Point2D &c) {
  return (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x);
}

double dotProduct(const Point2D &a, const Point2D &b) {
  return a.x * b.x + a.y * b.y;
}

bool lineIntersection(const Point2D &p1_start, const Point2D &p1_end,
                      const Point2D &p2_start, const Point2D &p2_end,
                      Point2D &intersect) {
  double p0_x, p0_y, p1_x, p1_y;
  double p2_x, p2_y, p3_x, p3_y;
  p0_x = p1_start.x;
  p0_y = p1_start.y;
  p1_x = p1_end.x;
  p1_y = p1_end.y;

  p2_x = p2_start.x;
  p2_y = p2_start.y;
  p3_x = p2_end.x;
  p3_y = p2_end.y;

  double s1_x, s1_y, s2_x, s2_y;
  s1_x = p1_x - p0_x;
  s1_y = p1_y - p0_y;
  s2_x = p3_x - p2_x;
  s2_y = p3_y - p2_y;

  double s, t;
  s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) /
      (-s2_x * s1_y + s1_x * s2_y);
  t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) /
      (-s2_x * s1_y + s1_x * s2_y);

  if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
    // Collision detected
    intersect.x = p0_x + (t * s1_x);
    intersect.y = p0_y + (t * s1_y);
    return true;
  }

  return false;  // No collision
}

bool isProjectedPointOnLine(const Point2D &line_start, const Point2D &line_end,
                            const Point2D &p) {
  Point2D e1(line_end.x - line_start.x, line_end.y - line_start.y);
  double recArea = dotProduct(e1, e1);
  Point2D e2(p.x - line_start.x, p.y - line_start.y);
  double val = dotProduct(e1, e2);

  /*
   MMa Nov 3,2016, I'm not sure if it's because the inprecise of the calculation
   or something wrong with the global path but sometime this function fails to
   find the projected point on a segment even if it's very obvious that the
   point is on the path. It failed when the val is smaller but close to 0 and
   val smaller but close to recArea. The problem is fixed by adding some margin
   to the val but not sure if this is the correct way.
   Note. The bug can be reproduced using the global path[20160810_frontlot.csv]
  */
  // return (val>=0 && val <= recArea);
  return (val >= 0 && val <= recArea);
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
  double normalLength = hypot(end.x - start.x, end.y - start.y);
  double distance = (double)((point.x - start.x) * (end.y - start.y) -
                             (point.y - start.y) * (end.x - start.x)) /
                    normalLength;
  return fabs(distance);
}

bool isPointInRect(const Rect r1, const Point2D p) {
  int i = 0;
  i += judgeDirection(r1.p[0], r1.p[1], p);
  i += judgeDirection(r1.p[1], r1.p[2], p);
  i += judgeDirection(r1.p[2], r1.p[3], p);
  i += judgeDirection(r1.p[3], r1.p[0], p);
  if (i == (-4) || i == (-3)) return true;
  return false;
}

// ---------------------------- 3D Point functions ----------------------------
double distance(const Point3D &p1, const Point3D &p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
              (p1.z - p2.z) * (p1.z - p2.z));
}

double distance3D(const Point3D &p1, const Point3D &p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
              (p1.z - p2.z) * (p1.z - p2.z));
}

// if judgeDirection>0,p is on left side of the line
// if judgeDirection<0,p is on right side of the line
int64_t judgeDirection(const Point3D &p1, const Point3D &p2, const Point3D &p) {
  double A, B, C, D;
  A = -(p2.y - p1.y);
  B = p2.x - p1.x;
  C = -(A * p1.x + B * p1.y);
  D = A * p.x + B * p.y + C;

  if (D <= 0.00001 && D >= -0.00001) return 0;
  if (D > 0) return 1;
  if (D < 0) return -1;
}

double perpDotProduct(const Point3D &a, const Point3D &b, const Point3D &c) {
  return (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x);
}

double dotProduct(const Point3D &a, const Point3D &b) {
  return a.x * b.x + a.y * b.y;
}

bool lineIntersection(const Point3D &p1_start, const Point3D &p1_end,
                      const Point3D &p2_start, const Point3D &p2_end,
                      Point3D &intersect) {
  double p0_x, p0_y, p1_x, p1_y;
  double p2_x, p2_y, p3_x, p3_y;
  p0_x = p1_start.x;
  p0_y = p1_start.y;
  p1_x = p1_end.x;
  p1_y = p1_end.y;

  p2_x = p2_start.x;
  p2_y = p2_start.y;
  p3_x = p2_end.x;
  p3_y = p2_end.y;

  double s1_x, s1_y, s2_x, s2_y;
  s1_x = p1_x - p0_x;
  s1_y = p1_y - p0_y;
  s2_x = p3_x - p2_x;
  s2_y = p3_y - p2_y;

  double s, t;
  s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) /
      (-s2_x * s1_y + s1_x * s2_y);
  t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) /
      (-s2_x * s1_y + s1_x * s2_y);

  if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
    // Collision detected
    intersect.x = p0_x + (t * s1_x);
    intersect.y = p0_y + (t * s1_y);
    return true;
  }

  return false;  // No collision
}

bool isProjectedPointOnLine(const Point3D &line_start, const Point3D &line_end,
                            const Point3D &p) {
  Point3D e1(line_end.x - line_start.x, line_end.y - line_start.y, 0);
  double recArea = dotProduct(e1, e1);
  Point3D e2(p.x - line_start.x, p.y - line_start.y, 0);
  double val = dotProduct(e1, e2);

  // MMa Nov 3,2016, I'm not sure if it's because the inprecise of the
  // calculation or
  // something wrong with the global path but sometime this function fails to
  // find the
  // projected point on a segment even if it's very obvious that the point is on
  // the
  // path. It failed when the val is smaller but close to 0 and val smaller but
  // close
  // to recArea. The problem is fixed by adding some margin to the val but not
  // sure if
  // this is the correct way.
  // Note. The bug can be reproduced using the global path
  // [20160810_frontlot.csv]
  // return (val>=0 && val <= recArea);
  return (val + 0.1 >= 0 && val <= recArea + 0.01);
}

Point3D getProjectedPointOnLine(const Point3D &line_start,
                                const Point3D &line_end, const Point3D &p) {
  // get dot product of e1, e2
  Point3D e1(line_end.x - line_start.x, line_end.y - line_start.y, 0);
  Point3D e2(p.x - line_start.x, p.y - line_start.y, 0);
  double val = dotProduct(e1, e2);

  // get squared length of e1
  double len2 = e1.x * e1.x + e1.y * e1.y;
  return Point3D((line_start.x + (val * e1.x) / len2),
                 (line_start.y + (val * e1.y) / len2), 0);
}

bool isInRange2D(const Point3D &a, const Point3D &b, double range) {
  return distance2D(a, b) <= range;
}

bool isSamePoint2D(const Point3D &a, const Point3D &b, double tolerence) {
  // In meters, ignore z
  return distance2D(a, b) <= tolerence;
}

double distance(const PolyLine3D &path, const int start, const int end) {
  if (path.size() < 2 || start < 0 || start + 1 >= end || start >= path.size())
    return 0;

  double dist = 0;
  for (int i = start; i + 1 < path.size() && i + 1 < end; i++)
    dist += distance(path[i], path[i + 1]);
  return dist;
}

double distance2D(const PolyLine3D &path, const int start, const int end) {
  if (path.size() < 2 || start < 0 || start + 1 >= end || start >= path.size())
    return 0;

  double dist = 0;
  for (int i = start; i + 1 < path.size() && i + 1 < end; i++)
    dist += distance2D(path[i], path[i + 1]);
  return dist;
}

}  // end of namespace geometry
