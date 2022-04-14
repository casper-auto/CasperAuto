#include <map>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "speed_planner_lib/log.h"
#include "speed_planner_lib/mpc.h"
#include "speed_planner_lib/st_utils.h"
#include "speed_planner_lib/variable_vector.h"

using namespace std;

using CppAD::AD;
using namespace std;

namespace planning {

/**
 * @class FGEval
 * @brief A data structure for evaluation function representing and computing.
 */
class FGEval {
public:
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  FGEval(MPCConfig *config, Vehicle *vehicle)
      : config_m(config), vehicle_m(vehicle) {}

  void operator()(ADvector &fg_raw, ADvector &vars_raw) {
    //**************************************************************************
    // `fg` is a vector containing the cost and constraints.
    // `vars` is a vector containing the variable values (state & actuators).
    //**************************************************************************

    // The cost is stored in the first element of `fg`.
    // Any additions to the cost should be added to `cost`.
    AD<double> &cost = fg_raw[0] = 0;

    //**************************************************************************
    // Reference State Cost
    //   Define the cost related the reference state and anything you think
    //   may be beneficial. */
    //**************************************************************************

    const size_t &n_steps = config_m->n_steps;
    const double dt = config_m->dt;

    VariableVector<AD<double>, ADvector> fg(n_steps, fg_raw, 1);
    VariableVector<AD<double>, ADvector> vars(n_steps, vars_raw);

    // Cost Calculation
    // Maximizing the displacement
    double max_displacement = n_steps * dt * vehicle_m->GetSpeedLimit();
    for (size_t t = 1; t < n_steps; t++)
      cost += config_m->displacement_weight * (max_displacement - vars.s[t]);

    // maximiz velocity
    for (size_t t = 1; t < n_steps; t++) {
      cost +=
          config_m->vel_weight * (config_m->s_dot_max - vars.s_dot[n_steps]);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < n_steps; t++) {
      cost += config_m->acc_weight * CppAD::pow(vars.brake[t], 2);
      cost += config_m->acc_weight * CppAD::pow(vars.throttle[t], 2);
    }

    // Prevent the use of brake and acceleration at same time
    for (size_t t = 0; t < n_steps; t++) {
      cost +=
          config_m->throttle_brake_weight * vars.brake[t] * vars.throttle[t];
    }

    // Minimize throttle_dot
    for (size_t t = 1; t < n_steps; t++) {
      cost += config_m->acc_dot_weight *
              CppAD::pow(vars.throttle[t] - vars.throttle[t - 1], 2);
    }

    // Minimize brake_dot
    for (size_t t = 1; t < n_steps; t++) {
      cost += config_m->acc_dot_weight *
              CppAD::pow(vars.brake[t] - vars.brake[t - 1], 2);
    }

    //**************************************************************************
    // Setup Model Constraints
    //**************************************************************************

    // From Veh state and odometry
    fg.s[0] = vars.s[0];
    fg.s_dot[0] = vars.s_dot[0];
    // fg.s_dotdot[0] = vars.s_dotdot[0];

    // current actuator values
    // fg.throttle[0] = vars.throttle[0];
    // fg.brake[0] = vars.brake[0];

    // The rest of the constraints
    fg.s_dotdot[0] = vars.s_dotdot[0] - (vars.throttle[0] - vars.brake[0]);
    for (unsigned int t = 1; t < n_steps; t++) {
      fg.s[t] =
          vars.s[t] - (vars.s[t - 1] + vars.s_dot[t - 1] * dt +
                       0.5 * vars.s_dotdot[t - 1] * dt * dt +
                       (vars.throttle_dot[t - 1] - vars.brake_dot[t - 1]) * dt *
                           dt * dt / 6);
      fg.s_dot[t] =
          vars.s_dot[t] -
          (vars.s_dot[t - 1] + vars.s_dotdot[t - 1] * dt +
           0.5 * (vars.throttle_dot[t - 1] - vars.brake_dot[t - 1]) * dt * dt);
      fg.s_dotdot[t] = vars.s_dotdot[t] - (vars.throttle[t] - vars.brake[t]);
      fg.brake[t] =
          vars.brake[t] - (vars.brake[t - 1] + vars.brake_dot[t - 1] * dt);
      fg.throttle[t] = vars.throttle[t] -
                       (vars.throttle[t - 1] + vars.throttle_dot[t - 1] * dt);
    }
  }

private:
  // MPC configuration parameters
  MPCConfig *config_m;
  // Vehicle state variables
  Vehicle *vehicle_m;
};

/**
 * @class MPC
 * @brief Apply Model Predictive Control.
 */
MPC::MPC(Vehicle *vehicle) : vehicle_m(vehicle) {
  DEBUG_MSG("Constructor of MPC");
}

void MPC::SetConfiguration(MPCConfig *config) {
  config_m = config;
  InitMPC();
  DEBUG_MSG("Set of MPC is done");
}

MPC::~MPC() {
  if (cost_log_open_flag_m)
    std::fclose(file_cost_log_m);
  if (long_log_open_flag_m)
    std::fclose(file_mpc_long_log_m);
  if (short_log_open_flag_m)
    std::fclose(file_mpc_short_log_m);
  DEBUG_MSG("Destructor of MPC");
}

void MPC::InitMPC() {
  DEBUG_MSG("INIT MPC");

  // actuator data
  throttles_m.clear();
  brakes_m.clear();

  velocities_m.clear();
  accelerations_m.clear();

  prediction_m.clear();

  mpc_itteration_m = 0;
  cost_log_open_flag_m = false;
  long_log_open_flag_m = false;
  short_log_open_flag_m = false;

  DEBUG_MSG("INIT MPC done");
}

void MPC::Solve(std::vector<STCell> st_path, std::map<int, double> speed_map,
                std::vector<STConfig> &path, double &cost) {
  if (config_m == NULL) {
    DEBUG_MSG("MPC::Error! MPC config is not set!!");
    return;
  }
  if (config_m->dt == 0) {
    DEBUG_MSG("MPC::Error! config_m->dt == 0!");
    return;
  }

  config_m->n_steps = st_path.size();
  const size_t &n_steps = config_m->n_steps;

  if (false) {
    DEBUG_MSG("n_steps = " << config_m->n_steps);
    DEBUG_MSG("dt = " << config_m->dt);
    DEBUG_MSG("vel_weight = " << config_m->vel_weight);
    DEBUG_MSG("acc_weight = " << config_m->acc_weight);
    DEBUG_MSG("acc_dot_weight = " << config_m->acc_dot_weight);
    DEBUG_MSG("default_limit = " << config_m->default_limit);
    DEBUG_MSG("throttle_max = " << vehicle_m->GetThrottleMax());
    DEBUG_MSG("throttle_d_max = " << vehicle_m->GetThrottleDotMax());
    DEBUG_MSG("throttle_d_min = " << vehicle_m->GetThrottleDotMin());
    DEBUG_MSG("brake_max = " << vehicle_m->GetBrakeMax());
    DEBUG_MSG("brake_d_max = " << vehicle_m->GetBrakeDotMax());
    DEBUG_MSG("brake_d_min = " << vehicle_m->GetBrakeDotMin());
    DEBUG_MSG("s_dot_max = " << vehicle_m->GetSpeedLimit());
  }

  typedef CPPAD_TESTVECTOR(double) Dvector;

  // const double& s = 0.0;
  // const double& s_dot = vehicle_m->GetVelocity();
  // const double& s_dotdot = vehicle_m->GetAcceleration();
  double s = 0.0;
  double s_dot = std::max(
      0.0, std::min(vehicle_m->GetSpeedLimit(), vehicle_m->GetVelocity()));
  double s_dotdot = vehicle_m->GetAcceleration();
  if (s_dot + s_dotdot * config_m->dt < 0.0) {
    s_dotdot = -s_dot / config_m->dt;
  }

  if (s_dot + s_dotdot * config_m->dt > vehicle_m->GetSpeedLimit()) {
    s_dotdot = (vehicle_m->GetSpeedLimit() - s_dot) / config_m->dt;
  }

  double current_brake = 0.0;
  double current_throttle = 0.0;
  if (s_dotdot > 0) {
    current_throttle = s_dotdot;
  } else if (s_dotdot < 0) {
    current_brake = -s_dotdot;
  }

  DEBUG_MSG("MPC::current config: s_d:" << s_dot << ", s_dd:" << s_dotdot
                                        << ", th:" << current_throttle
                                        << ", b:" << current_brake);

  VariableVector<double, Dvector> vars(n_steps);
  if (solution_m.x.size() == 0) {
    last_variable_size_m = n_steps;
    for (size_t i = 0; i < vars.raw_data.size(); i++) {
      vars.raw_data[i] = 0.0;
    }
  } else {
    size_t size = std::min(n_steps, last_variable_size_m);
    VariableVector<double, Dvector> solution(last_variable_size_m,
                                             solution_m.x);
    for (size_t i = 0; i < size - 1; i++) {
      vars.s[i] = solution.s[i + 1];
      vars.s_dot[i] = solution.s_dot[i + 1];
      vars.s_dotdot[i] = solution.s_dotdot[i + 1];
      vars.throttle[i] = solution.throttle[i + 1];
      vars.brake[i] = solution.brake[i + 1];
    }
    last_variable_size_m = n_steps;
  }

  // Set the initial variable values
  // Input from prediction of vehicle state based on veh_state and veh_odometry
  vars.s[0] = s;
  vars.s_dot[0] = s_dot;
  // vars.s_dotdot[0] = s_dotdot;
  // vars.brake[0] = current_brake;
  // vars.throttle[0] = current_throttle;

  //**************************************************************
  //* SET UPPER AND LOWER LIMITS OF VARIABLES
  //**************************************************************
  VariableVector<double, Dvector> vl(n_steps);
  VariableVector<double, Dvector> vu(n_steps);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < vl.raw_data.size(); i++) {
    vl.raw_data[i] = -config_m->default_limit;
    vu.raw_data[i] = config_m->default_limit;
  }

  for (size_t i = 0; i < n_steps; i++) {
    vl.s[i] = st_path[i].min_s;
    vu.s[i] = st_path[i].max_s;

    vl.s_dot[i] = 0.0;
    auto spd_it = speed_map.lower_bound(static_cast<int>(vars.s[i]));
    if (spd_it == speed_map.end())
      spd_it--;
    vu.s_dot[i] = spd_it->second;

    vl.s_dotdot[i] = -vehicle_m->GetBrakeMax();
    vu.s_dotdot[i] = vehicle_m->GetThrottleMax();

    // Actuators
    vl.throttle[i] = 0.0;
    vu.throttle[i] = vehicle_m->GetThrottleMax();
    vl.throttle_dot[i] = vehicle_m->GetThrottleDotMin();
    vu.throttle_dot[i] = vehicle_m->GetThrottleDotMax();

    vl.brake[i] = 0.0;
    vu.brake[i] = vehicle_m->GetBrakeMax();
    vl.brake_dot[i] = vehicle_m->GetBrakeDotMin();
    vu.brake_dot[i] = vehicle_m->GetBrakeDotMax();
  }

  // limits from the previous acceleration
  vl.throttle[0] =
      std::max(0.0, current_throttle +
                        vehicle_m->GetThrottleDotMin() * config_m->planning_dt);
  vu.throttle[0] = std::min(vehicle_m->GetThrottleMax(),
                            current_throttle + vehicle_m->GetThrottleDotMax() *
                                                   config_m->planning_dt);
  vl.brake[0] = std::max(0.0, current_throttle + vehicle_m->GetBrakeDotMin() *
                                                     config_m->planning_dt);
  vu.brake[0] = std::min(vehicle_m->GetBrakeMax(),
                         current_throttle + vehicle_m->GetBrakeDotMax() *
                                                config_m->planning_dt);

  //**************************************************************
  //* SET UPPER AND LOWER LIMITS OF CONSTRAINTS
  //**************************************************************
  // All of these should be 0 except the initial state indices.
  VariableVector<double, Dvector> cl(n_steps);
  VariableVector<double, Dvector> cu(n_steps);
  for (size_t i = 0; i < cl.raw_data.size(); i++) {
    cl.raw_data[i] = 0;
    cu.raw_data[i] = 0;
  }

  //
  for (size_t i = 0; i < cl.raw_data.size(); i++) {
    cl.s_dotdot[i] = 0.0;
    cu.s_dotdot[i] = 0.0;
  }

  // Constraint from veh and odometry
  cl.s[0] = s;
  cu.s[0] = s;
  cl.s_dot[0] = s_dot;
  cu.s_dot[0] = s_dot;

  printf("------------------------------------------------------------\n");
  printf("  Val =  [%f]\n", s_dot);
  // cl.s_dotdot[0] = s_dotdot;
  // cu.s_dotdot[0] = s_dotdot;

  // Initialize Previous Actuator constraints
  // cl.throttle[0] = current_throttle;
  // cu.throttle[0] = current_throttle;
  // cl.brake[0] = current_brake;
  // cu.brake[0] = current_brake;

  //****************************************************************************
  // Solve as an optimization problem using IPOPT
  //****************************************************************************

  // Object that computes objective and constraints
  FGEval fg_eval(config_m, vehicle_m);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // Maximum limit for computation.
  // options += "Integer max_iter     " + config_m->max_iteration + "\n";
  // options += "Numeric tol          " + config_m->tolerance + "\n";
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  // CppAD::ipopt::solve_result<Dvector> solution;
  solution_m.x.clear();
  solution_m.zl.clear();
  solution_m.zu.clear();
  solution_m.g.clear();
  solution_m.lambda.clear();

  // solve the problem
  CppAD::ipopt::solve<Dvector, FGEval>(options, vars.raw_data, vl.raw_data,
                                       vu.raw_data, cl.raw_data, cu.raw_data,
                                       fg_eval, solution_m);

  // Check some of the solution values
  bool ok = true;
  ok &= solution_m.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Analyze the failure case
  if (!ok) {
    static std::string status[] = {"not_defined",
                                   "success",
                                   "maxiter_exceeded",
                                   "stop_at_tiny_step",
                                   "stop_at_acceptable_point",
                                   "local_infeasibility",
                                   "user_requested_stop",
                                   "feasible_point_found",
                                   "diverging_iterates",
                                   "restoration_failure",
                                   "error_in_step_computation",
                                   "invalid_number_detected",
                                   "too_few_degrees_of_freedom",
                                   "internal_error",
                                   "unknown"};
    printf("------------------------------------------------------------\n");
    printf("  Solver failed. Error type [%s]\n",
           status[solution_m.status].c_str());
    printf("------------------------------------------------------------\n");
  }

  VariableVector<double, Dvector> solution(n_steps, solution_m.x);

  if (solution_m.status == CppAD::ipopt::solve_result<Dvector>::success ||
      solution_m.status == CppAD::ipopt::solve_result<Dvector>::maxiter_exceeded ||
      solution_m.status ==
          CppAD::ipopt::solve_result<Dvector>::stop_at_acceptable_point ||
      solution_m.status == CppAD::ipopt::solve_result<Dvector>::stop_at_tiny_step) {
    cost_m = solution_m.obj_value;

    // Statistic Test
    if (solution_m.status == CppAD::ipopt::solve_result<Dvector>::maxiter_exceeded) {
      DEBUG_MSG("-------- maxiter_exceeded --------");
    }

    if (solution_m.status ==
        CppAD::ipopt::solve_result<Dvector>::stop_at_acceptable_point) {
      DEBUG_MSG("-------- stop_at_acceptable_point --------");
    }

    if (solution_m.status == CppAD::ipopt::solve_result<Dvector>::stop_at_tiny_step) {
      DEBUG_MSG("-------- stop_at_tiny_step --------");
    }

    // DEBUG_MSG("s: ");
    // for (unsigned int i = 0; i < n_steps; i++) {
    //   DEBUG_MSG(std::setprecision(4) << solution.s[i] << ", ");
    // }
    //
    // DEBUG_MSG("s_dot: ");
    // for (unsigned int i = 0; i < n_steps; i++) {
    //   DEBUG_MSG(std::setprecision(4) << solution.s_dot[i] << ", ");
    // }
    //
    // DEBUG_MSG("s_dotdot: ");
    // for (unsigned int i = 0; i < n_steps; i++) {
    //   DEBUG_MSG(std::setprecision(4) << solution.s_dotdot[i] << ", ");
    // }
    //
    // DEBUG_MSG("throttle: ");
    // for (unsigned int i = 0; i < n_steps; i++) {
    //   DEBUG_MSG(std::setprecision(4) << solution.throttle[i] << ", ");
    // }
    //
    // DEBUG_MSG("brake   : ");
    // for (unsigned int i = 0; i < n_steps; i++) {
    //   DEBUG_MSG(std::setprecision(4) << solution.brake[i] << ", ");
    // }
    //
    // DEBUG_MSG("final cost_m = " << cost_m);

    //**************************************************************************
    // Logging
    //**************************************************************************

    double mpc_time = 0.0; // TODO
    // double mpc_time = ros::Time::now().toSec();

    // Log cost values
    if (config_m->mpc_cost_log_m) {
      if (!cost_log_open_flag_m) {
        std::time_t rawtime;
        struct std::tm *timeinfo;
        std::time(&rawtime);
        timeinfo = localtime(&rawtime);

        char aux_name_cost[100];
        std::strftime(aux_name_cost, 100, "COST_LOG_%y_%m_%d__%H_%M.csv",
                      timeinfo);
        file_cost_log_m = std::fopen(aux_name_cost, "w");
        fprintf(file_cost_log_m, "%s, %s, %s, %s, %s, %s, %s, %s, %s\n",
                "itteration", "time", "displacement_cost", "throttle_cost",
                "brake_cost", "throttle_brake", "throttle_dot_cost",
                "brake_dot_cost", "cost");
        cost_log_open_flag_m = true;
      }

      // Cost Calculation
      // The part of the cost based on the reference state.
      double displacement_cost = 0;
      double velocity_cost = 0;
      double throttle_cost = 0;
      double brake_cost = 0;
      double throttle_brake_cost = 0;
      double throttle_dot_cost = 0;
      double brake_dot_cost = 0;
      double total_cost = 0.0;

      // Maximize Displacement
      double max_displacement =
          n_steps * config_m->dt * vehicle_m->GetSpeedLimit();
      for (size_t t = 1; t < n_steps; t++) {
        displacement_cost +=
            config_m->displacement_weight * (max_displacement - solution.s[t]);
      }

      total_cost += displacement_cost;

      // Minimize the use of actuators.
      for (size_t t = 1; t < n_steps; t++) {
        brake_cost += config_m->acc_weight * CppAD::pow(solution.brake[t], 2);
        throttle_cost +=
            config_m->acc_weight * CppAD::pow(solution.throttle[t], 2);
      }
      total_cost += brake_cost;
      total_cost += throttle_cost;

      // Prevent the use of brake and acceleration at same time
      for (size_t t = 1; t < n_steps; t++) {
        throttle_brake_cost += config_m->throttle_brake_weight *
                               solution.brake[t] * solution.throttle[t];
      }
      total_cost += throttle_brake_cost;

      // Minimize throttle_dot
      for (size_t t = 1; t < n_steps; t++) {
        throttle_dot_cost +=
            config_m->acc_dot_weight *
            CppAD::pow(solution.throttle[t] - solution.throttle[t - 1], 2);
      }
      total_cost += throttle_dot_cost;

      // Minimize brake_dot
      for (size_t t = 1; t < n_steps; t++) {
        brake_dot_cost +=
            config_m->acc_dot_weight *
            CppAD::pow(solution.brake[t] - solution.brake[t - 1], 2);
      }
      total_cost += brake_dot_cost;

      fprintf(file_cost_log_m,
              "%i, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f\n",
              mpc_itteration_m, mpc_time, displacement_cost, throttle_cost,
              brake_cost, throttle_brake_cost, throttle_dot_cost,
              brake_dot_cost, total_cost);
    }

    if (config_m->mpc_long_log_m) {
      if (!long_log_open_flag_m) {
        std::time_t rawtime;
        struct std::tm *timeinfo;
        std::time(&rawtime);
        timeinfo = localtime(&rawtime);

        char aux_name_mpc[100];
        std::strftime(aux_name_mpc, 100,
                      "MPC_DATA_LONG_LOG_%y_%m_%d__%H_%M.csv", timeinfo);
        file_mpc_long_log_m = std::fopen(aux_name_mpc, "w");
        fprintf(file_mpc_long_log_m, "%s, %s, %s, %s, %s, %s, %s, %s, %s\n",
                "itteration", "Time", "s", "s_dot", "s_dotdot", "throttle",
                "brake", "throttle_d", "brake_d");
        long_log_open_flag_m = true;
      }

      for (unsigned int i = 0; i < n_steps; i++) {
        fprintf(file_mpc_long_log_m,
                "%i, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, "
                "%0.4f ,%0.4f \n",
                mpc_itteration_m, mpc_time, solution.s[i], solution.s_dot[i],
                solution.s_dotdot[i], solution.throttle[i], solution.brake[i],
                solution.throttle_dot[i], solution.brake_dot[i]);
      }
    }

    if (config_m->mpc_short_log_m) {
      if (!short_log_open_flag_m) {
        std::time_t rawtime;
        struct std::tm *timeinfo;
        std::time(&rawtime);
        timeinfo = localtime(&rawtime);

        char aux_name_short_mpc[100];
        std::strftime(aux_name_short_mpc, 100,
                      "MPC_DATA_SHORT_LOG_%y_%m_%d__%H_%M.csv", timeinfo);
        file_mpc_short_log_m = std::fopen(aux_name_short_mpc, "w");
        fprintf(file_mpc_short_log_m, "%s, %s, %s, %s, %s, %s, %s, %s, %s\n",
                "itteration", "Time", "s", "s_dot", "s_dotdot", "throttle",
                "brake", "throttle_d", "brake_d");

        short_log_open_flag_m = true;
      }
      fprintf(file_mpc_short_log_m,
              "%i, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f\n",
              mpc_itteration_m, mpc_time, solution.s[0], solution.s_dot[0],
              solution.s_dotdot[0], solution.throttle[1], solution.brake[1],
              solution.throttle_dot[0], solution.brake_dot[0]);
    }
  }

  // Clear old data
  brakes_m.clear();
  throttles_m.clear();
  velocities_m.clear();
  accelerations_m.clear();
  prediction_m.clear();
  path.clear();

  for (int i = 0; i < n_steps; i++) {
    throttles_m.push_back(solution.throttle[i]);
    brakes_m.push_back(solution.brake[i]);
    velocities_m.push_back(solution.s_dot[i]);
    accelerations_m.push_back(solution.s_dotdot[i]);
    prediction_m.push_back(solution.s[i]);

    auto spd_it = speed_map.lower_bound(static_cast<int>(solution.s[i]));
    if (spd_it == speed_map.end())
      spd_it--;

    STConfig st_config{solution.s[i], solution.s_dot[i], solution.s_dotdot[i],
                       (solution.throttle_dot[0] - solution.brake_dot[0]),
                       spd_it->second};

    path.push_back(st_config);
  }

  cost = solution_m.obj_value;
  mpc_itteration_m++;
}

// const std::vector<double>& MPC::Accs() const { return accelerations_m; }
// const std::vector<double>& MPC::Throttles() const { return throttles_m; }
// const std::vector<double>& MPC::Brakes() const { return brakes_m; }
// const std::vector<double>& MPC::Prediction() const { return prediction_m; }
// double MPC::Cost() { return cost_m; }

} // namespace planning
