#ifndef MPC_CONFIG_H
#define MPC_CONFIG_H

#include <math.h>
#include <string>

#include "speed_planner_lib/log.h"

namespace planning {

/**
 * @file mpc_config.h
 * @brief The class of mpc_config.
 */

/**
 * @class MPCConfig
 * @brief Encapsulation of MPC parameters.
 */

class MPCConfig {
public:
  /**
   * @brief Constructor
   */
  MPCConfig();

  /**
   * @brief Destructor
   */
  ~MPCConfig();

  // mpc data
  int n_steps;
  double dt;
  double planning_dt;
  double s_dot_max;
  std::string max_iteration;
  std::string tolerance;

  // Weights for the cost function
  double displacement_weight; // ToDo: To be updated
  double vel_weight;
  double acc_weight;
  double acc_dot_weight;
  double throttle_brake_weight;

  // limits
  double default_limit;

  // log_data
  bool mpc_short_log_m;
  bool mpc_long_log_m;
  bool mpc_cost_log_m;
};

} // namespace planning

#endif // MPC_CONFIG_H
