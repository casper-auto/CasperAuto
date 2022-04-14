#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include "speed_planner_lib/geometry_utils.h"
#include "speed_planner_lib/vec2D.h"

namespace planning {

typedef std::pair<int, int> pii;
typedef std::pair<float, float> pff;
typedef Vector2d<float> Vector2f;

class CollisionChecker {
public:
  static bool hasCollision(Vector2f pos1, Vector2f dir1, double length1,
                           double width1, Vector2f pos2, Vector2f dir2,
                           double length2, double width2);

  static std::vector<Vector2f> getBounds(Vector2f pos, Vector2f dir,
                                         float length, float width);
};

} // namespace planning

#endif // COLLISION_CHECKER_H
