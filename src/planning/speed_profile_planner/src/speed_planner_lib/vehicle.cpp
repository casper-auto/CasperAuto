/**
 * @file vehicle.cpp
 * @brief The class of Vehicle.
 */
#include "speed_planner_lib/vehicle.h"

namespace planning {

Vehicle::Vehicle()
    : x_m(0.0), y_m(0.0), theta_m(0.0), velocity_m(0.0), acceleration_m(0.0),
      throttle_max_m(2.0), brake_max_m(1.0), brake_dot_max_m(20.0),
      brake_dot_min_m(-20.0), throttle_dot_max_m(1.8),
      throttle_dot_min_m(-40.0), s_dot_max_m(20.0), s_dot_min_m(0.0),
      is_first_update_m(true), v_buffer_size_m(0), acc_buffer_size_m(0) {
  gettimeofday(&update_time_m, NULL);
  LoadVehicleParam();
}
void Vehicle::LoadVehicleParam() {
  // ToDo: Load vehicle param from XML file
  DEBUG_MSG("ToDo: Load Vehicle Param from xml file");
}
void Vehicle::SetCurrentConfig(double x, double y, double v) {
  x_m = x;
  y_m = y;
  if (is_first_update_m) {
    is_first_update_m = false;
    acceleration_m = 0.0;
    velocity_m = std::max(s_dot_min_m, std::min(s_dot_max_m, v));
    gettimeofday(&update_time_m, NULL);
  } else {
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    long dt, seconds, useconds;
    seconds = current_time.tv_sec - update_time_m.tv_sec;
    useconds = current_time.tv_usec - update_time_m.tv_usec;
    dt = ((seconds)*1000 + useconds / 1000.0) + 0.5;
    // DEBUG_MSG("dt: " << dt <<" milisec");
    if (dt > 0.0) {
      double v_in_filtered =
          (v_buffer_size_m * velocity_m + v) / (v_buffer_size_m + 1);
      // DEBUG_MSG("delta v: " << (v_in_filtered - velocity_m));
      double acc_temp = (v_in_filtered - velocity_m) / ((double)dt) * 1000.0;
      velocity_m = std::max(s_dot_min_m, std::min(s_dot_max_m, v_in_filtered));
      double acc_filtered = (acc_buffer_size_m * acceleration_m + acc_temp) /
                            (acc_buffer_size_m + 1);
      acceleration_m =
          std::max(-brake_max_m, std::min(throttle_max_m, acc_filtered));
    } else {
      double v_in_filtered =
          (v_buffer_size_m * velocity_m + v) / (v_buffer_size_m + 1);
      velocity_m = std::max(s_dot_min_m, std::min(s_dot_max_m, v_in_filtered));
    }
    // acceleration_m = std::max (acceleration_m, -brake_max_m);
    // acceleration_m = std::min (acceleration_m, throttle_max_m);
    gettimeofday(&update_time_m, NULL);
  }
  // ToDo: Update acceleration
}

void Vehicle::SetCurrentConfig(double x, double y, double v, double a) {
  this->x_m = x;
  this->y_m = y;
  this->velocity_m = v;
  this->acceleration_m = a;
}

double Vehicle::GetThrottleMax() { return throttle_max_m; }
double Vehicle::GetBrakeMax() { return brake_max_m; }
double Vehicle::GetBrakeDotMax() { return brake_dot_max_m; }
double Vehicle::GetBrakeDotMin() { return brake_dot_min_m; }
double Vehicle::GetThrottleDotMax() { return throttle_dot_max_m; }
double Vehicle::GetThrottleDotMin() { return throttle_dot_min_m; }
double Vehicle::GetSpeedLimit() { return s_dot_max_m; }
// double Vehicle::GetActuatorsLatency(){
//   return actuator_latency_m ;
//}

void Vehicle::SetSpeedLimit(double speed_limit) { s_dot_max_m = speed_limit; }
void Vehicle::SetThrottleDotMax(double throttle_dot_max) {
  throttle_dot_max_m = throttle_dot_max;
}
void Vehicle::SetBrakeDotMax(double brake_dot_max) {
  brake_dot_max_m = brake_dot_max;
}

void Vehicle::SetThrottleMax(double throttle_max) {
  throttle_max_m = throttle_max;
}
void Vehicle::SetBrakeMax(double brake_max) { brake_max_m = brake_max; }

// void Vehicle::SetActuatorsLatency(double latency){
//   actuator_latency_m = latency;
//}

void Vehicle::PrintVehicleConfiguration() {
  DEBUG_MSG("Vehicle config: x = " << x_m << ", y = " << y_m
                                   << ", v = " << velocity_m
                                   << " , acc = " << acceleration_m);
}

// void Vehicle::Update(double x, double y, double theta) {
//   this->x_ = x;
//   this->y_ = y;
//   this->theta_ = theta;
// }

// void Vehicle::Update(double x, double y, double theta, double velocity) {
//   this->x_ = x;
//   this->y_ = y;
//   this->theta_ = theta;
//   this->velocity_ = velocity;
// }

// void Vehicle::Update(double x, double y, double theta, double velocity,
// double acceleration) {
//   this->x_ = x;
//   this->y_ = y;
//   this->theta_ = theta;
//   this->velocity_ = velocity;
//   this->acceleration_ = acceleration_;
// }

} // namespace planning
