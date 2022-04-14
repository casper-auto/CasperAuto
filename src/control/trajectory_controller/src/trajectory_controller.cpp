#include "trajectory_controller.h"

////////////////////////////////////////////////////////////////////////////////
// Helper functions
////////////////////////////////////////////////////////////////////////////////

vector<double> linspace(double a, double b, int num, bool endpoint)
{
  if (a > b) return linspace(b, a, num, endpoint);

  vector<double> res;
  double offset = endpoint ? (b - a) / (num - 1) : (b - a) / num;
  double val = a;
  while (val < b) {
    res.push_back(val);
    val += offset;
  }
  return res;
}

double distance2D(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return sqrt(dx * dx + dy * dy);
}

double norm(vector<double> a) {
  double sum = 0;
  for (int i = 0; i < a.size(); i++) {
    sum += a[i] * a[i];
  }
  return sqrt(sum);
}

double cross(vector<double>& A, vector<double>& B) {
  return A[0] * B[1] - A[1] * B[0];
}

////////////////////////////////////////////////////////////////////////////////
// TrajectoryController class
////////////////////////////////////////////////////////////////////////////////

TrajectoryController::TrajectoryController(string control_method,
                                   double lookahead_dist_mpc,
                                   double lookahead_t_mpc) {
  m_vars;
  m_current_pose = {0.0, 0.0, 0.0};
  m_current_velocity = {0.0, 0.0, 0.0};
  m_desired_speed = 0;
  m_current_timestamp = 0.0;
  m_start_control_loop = false;
  m_set_commands = {0.0, 0.0, 0.0};  // throttle, steer, brake
  m_waypoints.resize(0);
  m_control_method = control_method;
  m_lookahead_t_mpc = lookahead_t_mpc;
  m_lookahead_dist_mpc = lookahead_dist_mpc;
}

void TrajectoryController::update_values(vector<double>& current_pose,
                                     vector<double>& current_velocity,
                                     double timestamp) {
  m_current_pose = current_pose;
  m_current_velocity = current_velocity;
  m_current_timestamp = timestamp;
}

void TrajectoryController::update_desired_speed() {
  if (m_closest_index < m_waypoints.size() - 1) {
    m_desired_speed = m_waypoints[m_closest_index][2];
  } else {
    m_desired_speed = m_waypoints[m_waypoints.size() - 1][2];
  }
}

void TrajectoryController::update_waypoints(vector<vector<double>>& waypoints) {
  m_waypoints.clear();
  m_waypoints.resize(waypoints.size());
  for (int i = 0; i < waypoints.size(); i++) {
    vector<double> tmp(waypoints[i].size());
    for (int j = 0; j < waypoints[0].size(); j++) {
      tmp[j] = waypoints[i][j];
    }
    m_waypoints[i] = tmp;
  }

  m_closest_index = get_closest_index(waypoints, m_current_pose);
}

int TrajectoryController::get_closest_index(vector<vector<double>> waypoints,
                                        vector<double> ego_state) {
  double closest_len = DBL_MAX;
  int closest_index = 0;
  for (int i = 0; i < waypoints.size(); i++) {
    double temp = pow(waypoints[i][0] - ego_state[0], 2) +
                  pow(waypoints[i][1] - ego_state[1], 2);
    if (temp < closest_len) {
      closest_len = temp;
      closest_index = i;
    }
  }
  m_closest_distance = sqrt(closest_len);

  return closest_index;
}

int TrajectoryController::get_goal_waypoint_index(double x, double y,
                                              vector<vector<double>> waypoints,
                                              double lookahead_distance) {
  for (int i = 0; i < waypoints.size(); i++) {
    double dist = distance2D(x, y, waypoints[i][0], waypoints[i][1]);
    if (abs(dist - lookahead_distance) <= 1e-2) {
      return i;
    }
  }
  return waypoints.size() - 1;
}

double TrajectoryController::get_alpha(vector<double> v1, vector<double> v2,
                                   double lookahead_distance) {
  double inner_prod = v1[0] * v2[0] + v1[1] * v2[1];
  return acos(inner_prod / lookahead_distance);
}

int TrajectoryController::get_steering_direction(vector<double> v1,
                                             vector<double> v2) {
  double cross_prod = v1[0] * v2[1] - v1[1] * v2[0];
  if (cross_prod >= 0) {
    return -1;
  }
  return 1;
}

double TrajectoryController::get_heading_error(vector<vector<double>> waypoints,
                                           double current_yaw) {
  double waypoint_delta_x = waypoints[1][0] - waypoints[0][0];
  double waypoint_delta_y = waypoints[1][1] - waypoints[0][1];
  double waypoint_heading = atan(waypoint_delta_y / waypoint_delta_x);
  double heading_error_mod = remainder((waypoint_heading - current_yaw), M_PI);
  if (heading_error_mod > M_PI / 2 && heading_error_mod < M_PI)
    heading_error_mod -= M_PI;
  return heading_error_mod;
}

double TrajectoryController::get_cte_heading_error(double v) {
  double kcte = 3.0;  // CONSTANT TO BE MOVED TO CONSTRUCTOR
  double proportional_cte_error = kcte * m_closest_distance;
  double cte_heading_error = atan(proportional_cte_error / v);
  double cte_heading_error_mod = remainder(cte_heading_error, M_PI);
  if (cte_heading_error_mod > M_PI / 2 && cte_heading_error_mod < M_PI)
    cte_heading_error_mod -= M_PI;
  return cte_heading_error_mod;
}

vector<double> TrajectoryController::get_predicted_wheel_location(
    double x, double y, double steering_angle, double yaw, double v) {
  double wheel_heading = yaw + steering_angle;
  double wheel_traveled_dist = v * (m_current_timestamp - m_vars["t_previous"]);
  return {x + wheel_traveled_dist * cos(wheel_heading),
          y + wheel_traveled_dist * sin(wheel_heading)};
}

void TrajectoryController::set_throttle(double input_throttle) {
  // Clamp the throttle command to valid bounds
  double throttle = max(min(input_throttle, 1.0), 0.0);
  m_set_commands[0] = throttle;
}

void TrajectoryController::set_steer(double input_steer_in_rad) {
  // Covnert radians to [-1, 1]
  double input_steer = rad_to_steer(input_steer_in_rad);

  // Clamp the steering command to valid bounds
  double steer = max(min(input_steer, 1.0), -1.0);
  m_set_commands[1] = steer;
}

void TrajectoryController::set_brake(double input_brake) {
  // Clamp the steering command to valid bounds
  double brake = max(min(input_brake, 1.0), 0.0);
  m_set_commands[2] = brake;
}

void TrajectoryController::reset_pid() {
  m_vars["prev_throttle"] = 0.0;
  m_vars["v_error_previous"] = 0.0;
}

double TrajectoryController::calculate_throttle(double t, double v,
                                            double v_desired) {
  // in this controller, we assume no braking, so brake_output is always 0
  // We use PID + feedforward method for longitudinal controller
  // the dynamic model is not used here, just pure tuning of the gains
  double kp = 0.5;  // CONSTANT TO BE MOVED TO CONSTRUCTOR
  double ki = 0.02;  // CONSTANT TO BE MOVED TO CONSTRUCTOR
  double kd = 0.05;  // CONSTANT TO BE MOVED TO CONSTRUCTOR

  double dt = t - m_vars["t_previous"];

  double v_error = v_desired - v;

  m_buffer_pid.push_back(v_error);
  if (m_buffer_pid.size() > 10) {
    m_buffer_pid.pop_front();
  }

  double acum_error = 0.0;
  for (int i = 0; i < m_buffer_pid.size(); i++) {
    acum_error += m_buffer_pid[i];
  }

  double k_term = kp * v_error;
  double i_term = ki * acum_error * dt;
  double d_term = kd * (v_error - m_vars["v_error_previous"]) / dt;

  // double throttle = m_vars["prev_throttle"] + k_term + i_term + d_term;
  double throttle =
      max(min(m_vars["prev_throttle"] + k_term + i_term + d_term, 1.0), 0.0);

  m_vars["prev_throttle"] = throttle;
  m_vars["v_error_previous"] = v_error;

  return throttle;
}

double TrajectoryController::calculate_steering(double t, double x, double y,
                                            double yaw,
                                            vector<vector<double>> waypoints,
                                            double v) {
  double steering = 0;

  // initialize parameters
  double kvf = 1.0;        // CONSTANT TO BE MOVED TO CONSTRUCTOR
  double wheelbase = 0.3;  // CONSTANT TO BE MOVED TO CONSTRUCTOR
  double kpp = 0.1;        // CONSTANT TO BE MOVED TO CONSTRUCTOR
  double kmpc = 0.3;       // CONSTANT TO BE MOVED TO CONSTRUCTOR

  if (m_control_method == "PurePursuit") {
    double lookahead_distance = kvf * v;
    int goal_idx = get_goal_waypoint_index(x, y, waypoints, lookahead_distance);
    vector<double> v1 = {waypoints[goal_idx][0] - x,
                         waypoints[goal_idx][1] - y};
    vector<double> v2 = {cos(yaw), sin(yaw)};
    double alpha = get_alpha(v1, v2, lookahead_distance);
    if (isnan(alpha)) alpha = m_vars["alpha_previous"];
    if (!isnan(alpha)) m_vars["alpha_previous"] = alpha;
    steering = get_steering_direction(v1, v2) *
               atan((2 * wheelbase * sin(alpha)) / (kpp * v));
  } else if (m_control_method == "Stanley") {
    vector<double> v1 = {waypoints[0][0] - x, waypoints[0][1] - y};
    vector<double> v2 = {cos(yaw), sin(yaw)};
    double heading_error = get_heading_error(waypoints, yaw);
    double cte_error =
        get_steering_direction(v1, v2) * get_cte_heading_error(v);
    steering = heading_error + cte_error;
  } else if (m_control_method == "MPC") {
    // parameters
    double lookahead_distance =
        max(m_lookahead_dist_mpc, v * m_lookahead_t_mpc);

    // goal_waypoint
    int goal_idx = waypoints.size() - 1;
    for (int i = 0; i < waypoints.size(); i++) {
      if (distance2D(x, y, waypoints[i][0], waypoints[i][1]) >
          lookahead_distance) {
        goal_idx = i;
        break;
      }
    }
    vector<double> goal_waypoint = waypoints[goal_idx];

    vector<double> steering_diff = linspace(-10, 10, 21);
    vector<double> steering_list(steering_diff.size());
    for (int i = 0; i < steering_list.size(); i++) {
      steering_list[i] =
          m_vars["steering_previous"] + steering_diff[i] * M_PI / 180;
    }

    double min_dist = distance2D(x, y, goal_waypoint[0], goal_waypoint[1]);
    steering = m_vars["steering_previous"];
    for (int i = 0; i < steering_list.size(); i++) {
      vector<double> predicted_wheel_location =
          get_predicted_wheel_location(x, y, steering_list[i], yaw, v);
      double dist_to_goal_waypoint =
          distance2D(predicted_wheel_location[0], predicted_wheel_location[1],
                     goal_waypoint[0], goal_waypoint[1]);
      if (dist_to_goal_waypoint < min_dist) {
        min_dist = dist_to_goal_waypoint;
        steering = steering_list[i];
      }
    }
  } else {
    steering = 0;
  }

  if (isnan(steering)) {
    steering = m_vars["steering_previous"];
  } else {
    m_vars["steering_previous"] = steering;
  }

  return steering;
}

void TrajectoryController::update_controls(double lag_throttle, double lag_brake,
                                       double lag_steering,
                                       double frequency_update,
                                       bool add_latency) {
  //////////////////////////////////////////////////////////////////////////////
  // Retrieve Data
  //////////////////////////////////////////////////////////////////////////////
  double x = m_current_pose[0];
  double y = m_current_pose[1];
  double yaw = m_current_pose[2];
  double v = m_current_velocity[2];

  update_desired_speed();

  double v_desired = m_desired_speed;
  double t = m_current_timestamp;

  double throttle_output = 0;
  double steer_output = 0;
  double brake_output = 0;

  if (abs(m_vars["t_previous"] - t) < 0.001) return;

  //////////////////////////////////////////////////////////////////////////////
  // Declare Usage Variable Here
  //////////////////////////////////////////////////////////////////////////////
  // create a persistent variable (not destroyed at each iteration).
  // This means that the value can be stored for use in the next
  // iteration of the control loop.
  if (!m_vars.count("v_previous")) m_vars["v_previous"] = 0.0;
  if (!m_vars.count("t_previous")) m_vars["t_previous"] = 0.0;
  if (!m_vars.count("prev_throttle")) m_vars["prev_throttle"] = 0.0;
  if (!m_vars.count("v_error_previous")) m_vars["v_error_previous"] = 0.0;
  if (!m_vars.count("alpha_previous")) m_vars["alpha_previous"] = 0.0;
  if (!m_vars.count("steering_previous")) m_vars["steering_previous"] = 0.0;

  // Skip the first frame to store previous values properly
  if (m_start_control_loop) {
    /* -----------------------------------------------------------------------
        Controller iteration code block.

        Controller Feedback Variables:
            x               : Current X position (meters)
            y               : Current Y position (meters)
            yaw             : Current yaw pose (radians)
            v               : Current forward speed (meters per second)
            t               : Current time (seconds)
            v_desired       : Current desired speed (meters per second)
                              (Computed as the speed to track at the
                              closest waypoint to the vehicle.)
            waypoints       : Current waypoints to track
                              (Includes speed to track at each x,y
                              location.)
                              Format: [[x0, y0, v0],
                                       [x1, y1, v1],
                                       ...
                                       [xn, yn, vn]]
                              Example:
                                  waypoints[2][1]:
                                  Returns the 3rd waypoint's y position

                                  waypoints[5]:
                                  Returns [x5, y5, v5] (6th waypoint)

        Controller Output Variables:
            throttle_output : Throttle output (0 to 1)
            steer_output    : Steer output (-1.22 rad to 1.22 rad)
            brake_output    : Brake output (0 to 1)
    ----------------------------------------------------------------------- */

    //////////////////////////////////////////////////////////////////////////
    // Implimentation of Longitudinal Ccontroller
    //////////////////////////////////////////////////////////////////////////
    double dt = 1 / frequency_update, v_desired_lag;
    if (!add_latency || lag_throttle < dt) {
      v_desired_lag = v_desired;
    } else {
      v_desired_lag = v + (v_desired - v) * dt / lag_throttle;
    }
    throttle_output = calculate_throttle(t, v, v_desired_lag);

    // Change these outputs with the longitudinal controller. Note that
    // brake_output is optional and is not required to pass the
    // assignment, as the car will naturally slow down over time.
    brake_output = 0;

    //////////////////////////////////////////////////////////////////////////
    // Implementation of Lateral Controller
    //////////////////////////////////////////////////////////////////////////
    vector<vector<double>> new_waypoints;
    for (int i = m_closest_index; i < m_waypoints.size(); i++) {
      new_waypoints.push_back(
          {m_waypoints[i][0], m_waypoints[i][1], m_waypoints[i][2]});
    }
    if (new_waypoints.size() > 1) {
      if (!add_latency || lag_steering < dt) {
        steer_output = calculate_steering(t, x, y, yaw, new_waypoints, v);
      } else {
        double steer_output_aux =
            calculate_steering(t, x, y, yaw, new_waypoints, v);
        steer_output = m_vars["steering_previous"] +
                       (steer_output_aux - m_vars["steering_previous"]) * dt /
                           lag_steering;
      }
    } else {
      throttle_output = 0;
      steer_output = 0;
      brake_output = 1;
    }

    // Special case for stopping
    if (v_desired < 0.1) {
      throttle_output = 0;
      steer_output = 0;
      brake_output = 1;
    }

    //////////////////////////////////////////////////////////////////////////
    // Set Controls Output
    //////////////////////////////////////////////////////////////////////////
    set_throttle(throttle_output);  // in percent (0 to 1)
    set_steer(-steer_output);       // in rad (-1.22 to 1.22)
    set_brake(brake_output);        // in percent (0 to 1)

    // Debug: Check the desired speed profile.
    /*cout << fixed << setprecision(2) << m_current_timestamp << ", "
                  << setprecision(4) << m_desired_speed << ", " << -steer_output
                  << endl;*/
  }

  //////////////////////////////////////////////////////////////////////////
  // STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
  //////////////////////////////////////////////////////////////////////////
  m_vars["v_previous"] = v;  // Store forward speed to be used in next step
  m_vars["t_previous"] = t;  // Store forward speed to be used in next step

  m_start_control_loop = true;
}
