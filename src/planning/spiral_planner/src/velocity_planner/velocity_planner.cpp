#include "velocity_planner/velocity_planner.h"

namespace planning {

///////////////////////////////////////////////////////////////////////////////
// class VelocityPlanner
///////////////////////////////////////////////////////////////////////////////

VelocityPlanner::VelocityPlanner(double t_gap, double a_max, double slow_v) {
  time_gap_ = t_gap;
  acc_max_ = a_max;
  slow_speed_ = slow_v;
  prev_profile_ = {Waypoint2D()};
}

double VelocityPlanner::getOpenloopSpeed(double timestep) {
  if (prev_profile_.size() == 1) {
    return prev_profile_[0].vel;
  }

  if (timestep < 1e-4) {
    return prev_profile_[0].vel;
  }

  for (int i = 0; i < prev_profile_.size() - 1; i++) {
    double distance_step =
        distance2D(prev_profile_[i].position, prev_profile_[i + 1].position);
    double velocity = prev_profile_[i].vel;
    double delta_time = distance_step / velocity;
    if (delta_time > timestep) {
      double v1 = prev_profile_[i].vel;
      double v2 = prev_profile_[i + 1].vel;
      double delta_v = v2 - v1;
      double interpolation_ratio = timestep / delta_time;
      return v1 + interpolation_ratio * delta_v;
    } else {
      timestep -= delta_time;
    }
  }

  return prev_profile_[prev_profile_.size() - 1].vel;
}

std::vector<Waypoint2D> VelocityPlanner::computeVelocityProfile(
    std::vector<Pose2D> &path, double desired_speed, State &current_state,
    State &leader_state, bool decelerate_to_stop, bool follow_lead_vehicle) {
  std::vector<Waypoint2D> profile;
  double start_speed = current_state.vel;
  // Generate a trapezoidal profile to decelerate to stop.
  if (decelerate_to_stop) {
    profile = decelerateProfile(path, start_speed);
  }
  // If we need to follow the lead vehicle, make sure we decelerate to its
  // speed by the time we reach the time gap point.
  else if (follow_lead_vehicle) {
    profile = followProfile(path, start_speed, desired_speed, leader_state);
  }
  // Otherwise, compute the profile to reach our desired speed.
  else {
    profile = nominalProfile(path, start_speed, desired_speed);
  }

  // Interpolate between the zeroth state and the first state.
  // This prevents the myopic controller from getting stuck at the zeroth
  // state.
  if (profile.size() > 1) {
    Waypoint2D interpolated_state(
        (profile[1].position.x - profile[0].position.x) * 0.1 +
            profile[0].position.x,
        (profile[1].position.y - profile[0].position.y) * 0.1 +
            profile[0].position.y,
        (profile[1].vel - profile[0].vel) * 0.1 + profile[0].vel);
    profile[0].position.x = interpolated_state.position.x;
    profile[0].position.y = interpolated_state.position.y;
    profile[0].vel = interpolated_state.vel;
  }

  // Save the planned profile for open loop speed estimation.
  prev_profile_ = profile;

  return profile;
}

std::vector<Waypoint2D>
VelocityPlanner::decelerateProfile(std::vector<Pose2D> &path,
                                   double start_speed) {
  std::vector<Waypoint2D> profile;

  // d = (v_f^2 - v_i^2) / (2 * a)
  double decel_distance = calcDistance(start_speed, slow_speed_, -acc_max_);
  double brake_distance = calcDistance(slow_speed_, 0, -acc_max_);

  // compute total path length
  double path_length = 0.0;
  for (int i = 0; i < path.size() - 1; i++) {
    path_length += distance2D(path[i].position, path[i + 1].position);
  }

  int stop_index = path.size() - 1;
  double temp_dist = 0.0;
  // Compute the index at which we should stop.
  while (stop_index > 0) {
    temp_dist +=
        distance2D(path[stop_index].position, path[stop_index - 1].position);
    stop_index -= 1;
  }

  if (brake_distance + decel_distance > path_length) {
    std::vector<double> speeds;
    double vf = 0.0;
    // The speeds past the stop line buffer should be zero.
    for (int i = path.size() - 1; i >= stop_index; i--) {
      speeds.insert(speeds.begin(), 0.0);
    }
    // The rest of the speeds should be a linear ramp from zero,
    // decelerating at -acc_max_.
    for (int i = stop_index; i >= 0; i--) {
      double dist = distance2D(path[i + 1].position, path[i].position);
      double vi = calcFinalSpeed(vf, -acc_max_, dist);
      // We don't want to have points above the starting speed
      // along our profile, so clamp to start_speed.
      if (vi > start_speed)
        vi = start_speed;
      speeds.insert(speeds.begin(), vi);
      vf = vi;
    }

    // Generate the profile, given the computed speeds.
    for (int i = 0; i < speeds.size(); i++) {
      profile.push_back(
          Waypoint2D(path[i].position.x, path[i].position.y, speeds[i]));
    }
  }
  // Otherwise, we will perform a full trapezoidal profile. The
  // brake_index will be the index of the path at which we start
  // braking, and the decel_index will be the index at which we stop
  // decelerating to our slow_speed_. These two indices denote the
  // endpoints of the ramps in our trapezoidal profile.
  else {
    int brake_index = stop_index;
    temp_dist = 0.0;
    // Compute the index at which to start braking down to zero.
    while ((brake_index > 0) && (temp_dist < brake_distance)) {
      temp_dist += distance2D(path[brake_index - 1].position,
                              path[brake_index].position);
      brake_index -= 1;
    }

    // Compute the index to stop decelerating to the slow speed.  This is
    // done by stepping through the points until accumulating
    // decel_distance of distance to said index, starting from the the
    // start of the path.
    int decel_index = 0;
    temp_dist = 0.0;
    while ((decel_index < brake_index) && (temp_dist < decel_distance)) {
      temp_dist += distance2D(path[decel_index].position,
                              path[decel_index + 1].position);
      decel_index += 1;
    }

    // The speeds from the start to decel_index should be a linear ramp
    // from the current speed down to the slow_speed_, decelerating at
    // -acc_max_.
    double vi = start_speed;
    double vf;
    for (int i = 0; i < decel_index; i++) {
      double dist = distance2D(path[i].position, path[i + 1].position);
      vf = calcFinalSpeed(vi, -acc_max_, dist);
      // We don't want to overshoot our slow_speed_, so clamp it to that.
      if (vf < slow_speed_) {
        vf = slow_speed_;
      }

      profile.push_back(Waypoint2D(path[i].position.x, path[i].position.y, vi));
      vi = vf;
    }

    // In this portion of the profile, we are maintaining our slow_speed_.
    for (int i = decel_index; i < brake_index; i++) {
      profile.push_back(Waypoint2D(path[i].position.x, path[i].position.y, vi));
    }

    // The speeds from the brake_index to stop_index should be a
    // linear ramp from the slow_speed_ down to the 0, decelerating at
    // -acc_max_.
    for (int i = brake_index; i < stop_index; i++) {
      double dist = distance2D(path[i].position, path[i + 1].position);
      vf = calcFinalSpeed(vi, -acc_max_, dist);
      profile.push_back(Waypoint2D(path[i].position.x, path[i].position.y, vi));
      vi = vf;
    }

    // The rest of the profile consists of our stop_line_buffer, so
    // it contains zero speed for all points.
    for (int i = stop_index; i < path.size(); i++) {
      profile.push_back(
          Waypoint2D(path[i].position.x, path[i].position.y, 0.0));
    }
  }

  return profile;
}

// Computes a profile for following a lead vehicle..
std::vector<Waypoint2D>
VelocityPlanner::followProfile(std::vector<Pose2D> &path, double start_speed,
                               double desired_speed, State &leader_state) {
  std::vector<Waypoint2D> profile;
  // Find the closest point to the lead vehicle on our planned path.
  int min_index = path.size() - 1;
  double min_dist = DBL_MAX;
  for (int i = 0; i < path.size(); i++) {
    double dist = distance2D(path[i].position, leader_state.position);
    if (dist < min_dist) {
      min_dist = dist;
      min_index = i;
    }
  }

  // Compute the time gap point, assuming our velocity is held constant at
  // the minimum of the desired speed and the ego vehicle's velocity, from
  // the closest point to the lead vehicle on our planned path.
  desired_speed = std::min(leader_state.vel, desired_speed);

  int ramp_end_index = min_index;
  double distance = min_dist;
  double distance_gap = desired_speed * time_gap_;
  while ((ramp_end_index > 0) && (distance > distance_gap)) {
    distance += distance2D(path[ramp_end_index - 1].position,
                           path[ramp_end_index].position);
    ramp_end_index -= 1;
  }

  // We now need to reach the ego vehicle's speed by the time we reach the
  // time gap point, ramp_end_index, which therefore is the end of our ramp
  // velocity profile.
  double decel_distance;
  if (desired_speed < start_speed) {
    decel_distance = calcDistance(start_speed, desired_speed, -acc_max_);
  } else {
    decel_distance = calcDistance(start_speed, desired_speed, acc_max_);
  }

  // Here we will compute the speed profile from our initial speed to the
  // end of the ramp.
  double vi = start_speed;
  double vf;
  for (int i = 0; i <= ramp_end_index && i < path.size() - 1; i++) {
    double dist = distance2D(path[i].position, path[i + 1].position);
    if (desired_speed < start_speed) {
      vf = calcFinalSpeed(vi, -acc_max_, dist);
    } else {
      vf = calcFinalSpeed(vi, acc_max_, dist);
    }
    profile.push_back(Waypoint2D(path[i].position.x, path[i].position.y, vi));
    vi = vf;
  }

  // Once we hit the time gap point, we need to be at the desired speed.
  // If we can't get there using a_max, do an abrupt change in the profile
  // to use the controller to decelerate more quickly.
  for (int i = ramp_end_index + 1; i < path.size(); i++) {
    profile.push_back(
        Waypoint2D(path[i].position.x, path[i].position.y, desired_speed));
  }

  return profile;
}

// Computes a profile for nominal speed tracking.
std::vector<Waypoint2D>
VelocityPlanner::nominalProfile(std::vector<Pose2D> &path, double start_speed,
                                double desired_speed) {
  std::vector<Waypoint2D> profile;
  // Compute distance travelled from start speed to desired speed using
  // a constant acceleration.
  double accel_distance;
  if (desired_speed < start_speed) {
    accel_distance = calcDistance(start_speed, desired_speed, -acc_max_);
  } else {
    accel_distance = calcDistance(start_speed, desired_speed, acc_max_);
  }

  // Here we will compute the end of the ramp for our velocity profile.
  // At the end of the ramp, we will maintain our final speed.
  int ramp_end_index = 0;
  double distance = 0.0;
  while ((ramp_end_index < path.size() - 1) && (distance < accel_distance)) {
    distance += distance2D(path[ramp_end_index].position,
                           path[ramp_end_index + 1].position);
    ramp_end_index += 1;
  }

  // Here we will actually compute the velocities along the ramp.
  double vi = start_speed;
  double vf;
  for (int i = 0; i < ramp_end_index; i++) {
    double dist = distance2D(path[i].position, path[i + 1].position);
    if (desired_speed < start_speed) {
      vf = calcFinalSpeed(vi, -acc_max_, dist);
      // clamp speed to desired speed
      if (vf < desired_speed) {
        vf = desired_speed;
      }
    } else {
      vf = calcFinalSpeed(vi, acc_max_, dist);
      // clamp speed to desired speed
      if (vf > desired_speed) {
        vf = desired_speed;
      }
    }

    profile.push_back(Waypoint2D(path[i].position.x, path[i].position.y, vi));
    vi = vf;
  }

  // If the ramp is over, then for the rest of the profile we should
  // track the desired speed.
  for (int i = ramp_end_index + 1; i < path.size(); i++) {
    profile.push_back({path[i].position.x, path[i].position.y, desired_speed});
  }

  return profile;
}

// Using d = (v_f^2 - v_i^2) / (2 * a), compute the distance
// required for a given acceleration/deceleration.
double VelocityPlanner::calcDistance(double v_i, double v_f, double a) {
  /**
  Computes the distance given an initial and final speed, with a constant
  acceleration.

  args:
      v_i: initial speed (m/s)
      v_f: final speed (m/s)
      a: acceleration (m/s^2)
  returns:
      d: the final distance (m)
  */
  return (v_f * v_f - v_i * v_i) / (2 * a);
}

// Using v_f = sqrt(v_i^2 + 2ad), compute the final speed for a given
// acceleration across a given distance, with initial speed v_i.
// Make sure to check the discriminant of the radical. If it is negative,
// return zero as the final speed.
double VelocityPlanner::calcFinalSpeed(double v_i, double a, double d) {
  /**
  Computes the final speed given an initial speed, distance travelled,
  and a constant acceleration.

  args:
      v_i: initial speed (m/s)
      a: acceleration (m/s^2)
      d: distance to be travelled (m)
  returns:
      v_f: the final speed (m/s)
  */
  double temp = v_i * v_i + 2 * d * a;
  if (temp < 0)
    return 0.0000001;
  else
    return sqrt(temp);
}

} // namespace planning
