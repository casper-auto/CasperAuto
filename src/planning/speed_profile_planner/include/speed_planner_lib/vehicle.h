/**
 * @file vehicle.h
 * @brief The class of Vehicle.
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include <cstddef>
#include <iostream>
#include <sys/time.h>

#include "speed_planner_lib/log.h"
#include "speed_planner_lib/point2d.h"

/**
 * @file vehicle.h
 * @brief The class of Vehicle.
 */

namespace planning {

/**
 * @class Vehicle
 *
 * @brief Implementation of the ego car. It encapsulates the ego car parameters
 * and it's kinematic.
 */
class Vehicle {
public:
  /**
   * @brief Constructor
   */
  Vehicle();

  /**
   * @brief Set current configuration of ego car.
   * @param x x value in \f$ m \f$
   * @param y y value in \f$ m \f$
   * @param v velocity in \f$ m/s \f$
   */
  void SetCurrentConfig(double x, double y, double v);
  /**
   * @brief Set current configuration of ego car.
   * @param x x value in \f$ m \f$
   * @param y y value in \f$ m \f$
   * @param v velocity in \f$ m/s \f$
   * @param a acceleration in \f$ m/s^2 \f$
   */
  void SetCurrentConfig(double x, double y, double v, double a);

  /**
   * @brief Return the position of the ego car.
   * @return position of the ego car
   */
  Point2D GetPosition() { return Point2D(x_m, y_m); }

  /**
   * @brief Return heading of the ego car.
   * @return heading of the ego car in radian
   */
  double GetHeading() { return theta_m; }

  /**
   * @brief Return current velocity.
   * @return velocity in \f$ m/s \f$
   */
  double GetVelocity() { return velocity_m; }

  /**
   * @brief Return current acceleration
   * @return acceleration in \f$ m/s^2 \f$.
   */
  double GetAcceleration() { return acceleration_m; }

  /**
   * @brief Return maximum throttle value.
   * @return max acceleration in \f$ m/s^2 \f$( max acceleration).
   */
  double GetThrottleMax();

  /**
   * @brief Return maximum Brake value.
   * @return max deceleration (positive value) in \f$ m/s^2 \f$ ( max
   * deceleration).
   */
  double GetBrakeMax();

  /**
   * @brief Return maximum brake rate.
   * @return maximum brake rate (positive value) in \f$ m/s^3 \f$.
   */
  double GetBrakeDotMax();

  /**
   * @brief Return brake rate in releasing the brake
   * @return minimum brake rate (negative value) in \f$ m/s^3 \f$.
   */
  double GetBrakeDotMin();

  /**
   * @brief Return maximum throttle rate
   * @return maximum throttle rate (positive value) in \f$ m/s^3 \f$.
   */
  double GetThrottleDotMax();

  /**
   * @brief Return acceleration rate in releasing the acceleration
   * @return acceleration rate (negative value) in \f$ m/s^3 \f$.
   */
  double GetThrottleDotMin();

  /**
   * @brief Return ego car speed limit.
   * @return ego car speed limit \f$ m/s \f$
   */
  double GetSpeedLimit();

  /**
   * @brief Set ego car speedlimit.
   * @param speed_limit speedlimit in \f$ m/s \f$
   */
  void SetSpeedLimit(double speed_limit);

  /**
   * @brief Set max throttle value (max acceleration)
   * @param throttle_max max acceleration in \f$ m/s^2 \f$
   */
  void SetThrottleMax(double throttle_max);

  /**
   * @brief Set max brake value (max deceleration)
   * @param brake_max max brake (positive value) in \f$ m/s^2 \f$
   */
  void SetBrakeMax(double brake_max);

  /**
   * @brief Set max jerk during acceleration
   * @param throttle_dot_max max jerk in \f$ m/s^3 \f$
   */
  void SetThrottleDotMax(double throttle_dot_max);

  /**
   * @brief Set max jerk during deceleration
   * @param brake_dot_max jerk in \f$ m/s^3 \f$ (positive value)
   */
  void SetBrakeDotMax(double brake_dot_max);

  /**
   * @brief Print vehicle config
   */
  void PrintVehicleConfiguration();

  //  const double GetWidth() const { return width_; }
  //  const double GetLength() const { return length_; }

private:
  /**
   * @brief Return heading of the ego car.
   * @return heading of the ego car
   */
  void LoadVehicleParam();
  double x_m;
  double y_m;
  double theta_m;
  double velocity_m;
  double acceleration_m;

  // Limits
  double throttle_max_m;
  double brake_max_m;
  double brake_dot_max_m;
  double brake_dot_min_m;
  double throttle_dot_max_m;
  double throttle_dot_min_m;
  double s_dot_max_m;
  double s_dot_min_m;

  // Latency
  // double actuator_latency_m;

  //
  bool is_first_update_m;
  int v_buffer_size_m;
  int acc_buffer_size_m;
  //  double update_time_m;
  struct timeval update_time_m;

  //  double width_;
  //  double length_;
};

} // namespace planning

#endif // VEHICLE_H
