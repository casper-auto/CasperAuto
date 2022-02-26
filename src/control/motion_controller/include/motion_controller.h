#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std;

////////////////////////////////////////////////////////////////////////////////
// Helper functions
////////////////////////////////////////////////////////////////////////////////

vector<double> linspace(double a, double b, int num = 50, bool endpoint = true);

double distance2D(double x1, double y1, double x2, double y2);

double norm(vector<double> a);

double cross(vector<double>& A, vector<double>& B);

////////////////////////////////////////////////////////////////////////////////
// MotionController class
////////////////////////////////////////////////////////////////////////////////

class MotionController {
 public:
  MotionController(string control_method, double lookahead_dist_mpc,
                   double lookahead_t_mpc);

  void update_values(vector<double>& current_pose, vector<double>& current_velocity, double timestamp);

  void update_desired_speed();

  void update_waypoints(vector<vector<double>>& waypoints);

  vector<double> get_commands() { return m_set_commands; }

  int get_closest_index(vector<vector<double>> waypoints,
                        vector<double> ego_state);

  void set_throttle(double input_throttle);

  void set_steer(double input_steer_in_rad);

  void set_brake(double input_brake);

  void reset_pid();

  void update_controls(double lag_throttle, double lag_brake,
                       double lag_steering, double frequency_update,
                       bool add_latency);

  void reset_all_vars() { m_vars.clear(); }

 private:
  string m_control_method;
  double m_lookahead_dist_mpc;
  double m_lookahead_t_mpc;

  unordered_map<string, double> m_vars;

  deque<double> m_buffer_pid;

  vector<double> m_current_pose;
  vector<double> m_current_velocity;

  double m_desired_speed;

  double m_current_timestamp;

  bool m_start_control_loop;

  vector<vector<double>> m_waypoints;

  int m_closest_index;

  double m_closest_distance;

  vector<double> m_set_commands;  // throttle, steer, brake

  double rad_to_steer(double steer_in_rad) {
    return steer_in_rad * 180.0 / 70.0 / M_PI;
  }

  int get_goal_waypoint_index(double x, double y,
                              vector<vector<double>> waypoints,
                              double lookahead_distance);

  double get_alpha(vector<double> v1, vector<double> v2,
                   double lookahead_distance);

  int get_steering_direction(vector<double> v1, vector<double> v2);

  double get_heading_error(vector<vector<double>> waypoints,
                           double current_yaw);

  double get_cte_heading_error(double v);

  vector<double> get_predicted_wheel_location(double x, double y,
                                              double steering_angle, double yaw,
                                              double v);

  double calculate_throttle(double t, double v, double v_desired);

  double calculate_steering(double t, double x, double y, double yaw,
                            vector<vector<double>> waypoints, double v);
};

#endif /* MOTION_CONTROLLER_H */
