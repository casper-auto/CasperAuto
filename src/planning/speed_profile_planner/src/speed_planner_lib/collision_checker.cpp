#include "speed_planner_lib/collision_checker.h"

namespace planning {

// http://www.gamedev.net/page/resources/_/technical/game-programming/2d-rotated-rectangle-collision-r2604
bool CollisionChecker::hasCollision(Vector2f pos1, Vector2f dir1,
                                    double length1, double width1,
                                    Vector2f pos2, Vector2f dir2,
                                    double length2, double width2) {
  Vector2f diff = pos2 - pos1;
  float dist = diff.Length();
  double radius1 = sqrt(pow(length1 / 2, 2) + pow(width1 / 2, 2));
  double radius2 = sqrt(pow(length2 / 2, 2) + pow(width2 / 2, 2));
  if (dist > radius1 + radius2)
    return false;

  std::vector<Vector2f> bounds1 = getBounds(pos1, dir1, length1, width1);
  std::vector<Vector2f> bounds2 = getBounds(pos2, dir2, length2, width2);
  Vector2f vec1 = bounds1[0] - bounds1[1];
  Vector2f vec2 = bounds2[0] - bounds2[1];
  std::vector<Vector2f> axis = {vec1, vec1.perpendicular(), vec2,
                                vec2.perpendicular()};

  for (const auto &vec : axis) {
    pff result = projectPoints(bounds1, vec);
    float minA = result.first;
    float maxA = result.second;
    result = projectPoints(bounds2, vec);
    float minB = result.first;
    float maxB = result.second;
    bool leftmostA = (minA <= minB) ? true : false;
    bool overlap = false;

    if (leftmostA && maxA >= minB)
      overlap = true;
    if (!leftmostA && maxB >= minA)
      overlap = true;
    if (!overlap)
      return false;
  }
  return true;
}

std::vector<Vector2f> CollisionChecker::getBounds(Vector2f pos, Vector2f dir,
                                                  float length, float width) {
  dir.normalized();
  Vector2f perp_dir = dir.perpendicular();

  std::vector<Vector2f> bounds;
  bounds.push_back(pos + dir * float(length / 2) + perp_dir * float(width / 2));
  bounds.push_back(pos + dir * float(length / 2) - perp_dir * float(width / 2));
  bounds.push_back(pos - dir * float(length / 2) + perp_dir * float(width / 2));
  bounds.push_back(pos - dir * float(length / 2) - perp_dir * float(width / 2));

  return bounds;
}

} // namespace planning
