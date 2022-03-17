#ifndef STATE_H
#define STATE_H

#include "geometry.hpp"

namespace planning {

struct State {
  geometry::Point2D position;
  double yaw;
  double vel;

  State() = default;

  State(double x, double y) {
    this->position = geometry::Point2D(x, y);
    this->yaw = 0;
    this->vel = 0;
  }

  State(double x, double y, double yaw) {
    this->position = geometry::Point2D(x, y);
    this->yaw = yaw;
    this->vel = 0;
  }

  State(double x, double y, double yaw, double vel) {
    this->position = geometry::Point2D(x, y);
    this->yaw = yaw;
    this->vel = vel;
  }
};

} // namespace planning

#endif // STATE_H
