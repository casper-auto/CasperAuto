#include <fstream>
#include <iomanip>
#include <iostream>

#include "speed_planner_lib/mpc_config.h"

namespace planning {

MPCConfig::MPCConfig()
    : n_steps(50), dt(1.0), planning_dt(0.1), s_dot_max(5.0),
      max_iteration("200"), tolerance("1e-6"), displacement_weight(10.0),
      vel_weight(100.0), acc_weight(1.0), acc_dot_weight(100.0),
      throttle_brake_weight(100.0), default_limit(1.0e19),
      mpc_short_log_m(false), mpc_long_log_m(false), mpc_cost_log_m(false) {
  DEBUG_MSG("Constructor of MPCConfig");
}

MPCConfig::~MPCConfig() { DEBUG_MSG("Destructor of MPCConfig"); }

} // namespace planning
