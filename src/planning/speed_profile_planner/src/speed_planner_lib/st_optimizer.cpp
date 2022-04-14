#include "speed_planner_lib/st_optimizer.h"

namespace planning {

STOptimizer::STOptimizer(Vehicle *vehicle)
    : vehicle_m(vehicle), mpc_m(vehicle) {
  mpc_config_m = new MPCConfig();
  mpc_m.SetConfiguration(mpc_config_m);
}

STOptimizer::STOptimizer(Vehicle *vehicle, MPCConfig *mpc_config)
    : vehicle_m(vehicle), mpc_m(vehicle) {
  mpc_config_m = mpc_config;
  mpc_m.SetConfiguration(mpc_config_m);
}

STOptimizer::~STOptimizer() {
  delete mpc_config_m;
  mpc_config_m = NULL;
}

Status STOptimizer::optimizePaths(std::vector<std::vector<STCell>> st_paths,
                                  std::vector<STConfig> &path) {
  DEBUG_MSG("STOptimizer::optimizePaths start");
  std::vector<std::vector<STConfig>> paths;
  std::vector<double> costs;
  double min_cost = std::numeric_limits<double>::max();
  int min_index = -1;
  int counter = 0;

  std::map<int, double> speed_map;
  for (int i = 0; i < path.size(); i++) {
    speed_map.insert(std::pair<int, double>(static_cast<int>(path[i].s),
                                            path[i].s_dot_limit));
  }
  DEBUG_MSG("speed limits");
  for (auto it = speed_map.begin(); it != speed_map.end(); ++it) {
    DEBUG_MSG("  " << it->first << ", " << it->second);
  }

  for (auto p : st_paths) {
    DEBUG_MSG("[Debug] ST Path");
    if (evaluatePath(p, speed_map)) {
      std::vector<STConfig> conf_path;
      double cost = 0.0;
      Status s = optimizePath(p, speed_map, conf_path, cost);
      DEBUG_MSG("  cost = " << cost);
      if (s == Status::OK()) {
        if (cost < min_cost) {
          min_cost = cost;
          min_index = counter;
        }
        paths.push_back(conf_path);
        costs.push_back(cost);
        counter++;
      }
    }
  }

  path.clear();
  if (counter == 0) {
    DEBUG_MSG("STOptimizer::optimizePaths end : Fail. Full Brake");
    STConfig init_conf;
    init_conf.s = 0.0;
    init_conf.s_dot = vehicle_m->GetVelocity();
    init_conf.s_dotdot = vehicle_m->GetAcceleration();
    init_conf.s_dotdotdot = -vehicle_m->GetBrakeDotMax();
    init_conf.s_dot_limit = 0.0;
    path.push_back(init_conf);
    for (int i = 0; i < mpc_config_m->n_steps - 1; i++) {
      STConfig conf;
      conf.s = 0.0;
      conf.s_dot = 0.0;
      conf.s_dotdot = 0.0;
      conf.s_dotdotdot = 0.0;
      conf.s_dot_limit = 0.0;
      path.push_back(conf);
    }
    return Status("Fail");
  } else {
    path = paths[min_index];
    DEBUG_MSG("STOptimizer::optimizePaths end : find solution "
              << min_index << " with cost of " << min_cost);
    return Status::OK();
  }
}

Status STOptimizer::optimizePath(std::vector<STCell> st_path,
                                 std::map<int, double> speed_map,
                                 std::vector<STConfig> &path, double &cost) {
  DEBUG_MSG("[Debug] STOptimizer::optimizePath start");
  struct timeval start, end;
  long mtime, seconds, useconds;
  gettimeofday(&start, NULL);
  mpc_m.Solve(st_path, speed_map, path, cost);
  gettimeofday(&end, NULL);
  seconds = end.tv_sec - start.tv_sec;
  useconds = end.tv_usec - start.tv_usec;
  mtime = ((seconds)*1000 + useconds / 1000.0) + 0.5;
  DEBUG_MSG("  Elapsed time: " << mtime << " msec, cost: " << cost);
  DEBUG_MSG("  STOptimizer::optimizePath end");
  return Status::OK();
}

bool STOptimizer::evaluatePath(std::vector<STCell> st_path,
                               std::map<int, double> speed_map) {
  DEBUG_MSG("[Debug] STOptimizer::evaluatePath");
  double max_feasible_velocity = vehicle_m->GetVelocity();
  double min_feasible_velocity = vehicle_m->GetVelocity();
  double max_feasible_s = 0.0;
  double min_feasible_s = 0.0;
  double dt = mpc_config_m->dt;
  for (unsigned int i = 1; i < st_path.size(); i++) {
    max_feasible_s += max_feasible_velocity * dt +
                      0.5 * vehicle_m->GetThrottleMax() * dt * dt;
    max_feasible_s = std::min(st_path[i].max_s, max_feasible_s);
    double min_delta_s =
        std::max(0.0, min_feasible_velocity * dt -
                          0.5 * vehicle_m->GetBrakeMax() * dt * dt);
    min_feasible_s += min_delta_s;
    min_feasible_s = std::max(st_path[i].min_s, min_feasible_s);
    if (max_feasible_s <= min_feasible_s) {
      DEBUG_MSG("========= STOptimizer::evaluatePath: Not feasible "
                << min_feasible_s << ", " << max_feasible_s);
      return false;
    }
    auto spd_it = speed_map.lower_bound(
        static_cast<int>((max_feasible_s + min_feasible_s) / 2.0));
    if (spd_it == speed_map.end())
      spd_it--;
    double speedlimit = spd_it->second;

    max_feasible_velocity += vehicle_m->GetThrottleMax() * dt;
    max_feasible_velocity = std::min(speedlimit, max_feasible_velocity);
    min_feasible_velocity += -vehicle_m->GetBrakeMax() * dt;
    min_feasible_velocity =
        std::min(speedlimit, std::max(0.0, min_feasible_velocity));
  }
  return true;
}

} // namespace planning
