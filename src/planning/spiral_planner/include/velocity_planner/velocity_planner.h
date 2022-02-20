#ifndef VELOCITY_PLANNER_H
#define VELOCITY_PLANNER_H

#include <cfloat>
#include <cmath>
#include <vector>

#include "geometry.hpp"
#include "state.h"

using namespace geometry;

namespace planning {

struct Waypoint2D {
  Point2D position;
  double vel;

  Waypoint2D() {
    this->position = Point2D(0, 0);
    this->vel = 0;
  }

  Waypoint2D(Point2D position, double vel) {
    this->position = position;
    this->vel = vel;
  }

  Waypoint2D(double x, double y, double vel) {
    this->position.x = x;
    this->position.y = y;
    this->vel = vel;
  }
};

///////////////////////////////////////////////////////////////////////////////
// class VelocityPlanner
///////////////////////////////////////////////////////////////////////////////

class VelocityPlanner {
public:
  VelocityPlanner() {}

  VelocityPlanner(double t_gap, double a_max, double slow_v);

  double getOpenloopSpeed(double timestep);

  std::vector<Waypoint2D>
  computeVelocityProfile(std::vector<Pose2D> &path, double desired_speed,
                         State &ego_state, State &leader_state,
                         bool decelerate_to_stop, bool follow_lead_vehicle);

private:
  double time_gap_;
  double acc_max_;
  double slow_speed_;

  std::vector<Waypoint2D> prev_profile_;

  std::vector<Waypoint2D> decelerateProfile(std::vector<Pose2D> &path,
                                            double start_speed);

  std::vector<Waypoint2D> followProfile(std::vector<Pose2D> &path,
                                        double start_speed,
                                        double desired_speed,
                                        State &leader_state);

  std::vector<Waypoint2D> nominalProfile(std::vector<Pose2D> &path,
                                         double start_speed,
                                         double desired_speed);

  // Using d = (v_f^2 - v_i^2) / (2 * a), compute the distance
  // required for a given acceleration/deceleration.
  double calcDistance(double v_i, double v_f, double a);

  // Using v_f = sqrt(v_i^2 + 2ad), compute the final speed for a given
  // acceleration across a given distance, with initial speed v_i.
  // Make sure to check the discriminant of the radical. If it is negative,
  // return zero as the final speed.
  double calcFinalSpeed(double v_i, double a, double d);
};

} // namespace planning

#endif /* VELOCITY_PLANNER_H */
