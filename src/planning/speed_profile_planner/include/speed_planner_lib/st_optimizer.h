/**
 * @file st_optimizer.h
 * @brief The class of STOptimizer.
 */

#ifndef ST_OPTIMIZER_H
#define ST_OPTIMIZER_H

#include <limits.h>
#include <vector>

#include "speed_planner_lib/log.h"
#include "speed_planner_lib/mpc.h"
#include "speed_planner_lib/mpc_config.h"
#include "speed_planner_lib/st_utils.h"
#include "speed_planner_lib/status.h"
#include "speed_planner_lib/vehicle.h"

namespace planning {

/**
 * @class STOptimizer
 * @brief Implementation of optimizer to generated speed profile. The class
 * convert a sequence of STCells to speed profile.
 */

class STOptimizer {
public:
  /**
   * @brief Constructor
   * @param vehicle pointer to ego vehicle
   */
  STOptimizer(Vehicle *vehicle);

  /**
   * @brief Constructor
   * @param vehicle pointer to ego vehicle
   * @param mpc_config pointer to MPC Configuration
   */
  STOptimizer(Vehicle *vehicle, MPCConfig *mpc_config);

  /**
   * @brief Destructor
   */
  ~STOptimizer();

  /**
   * @brief optimize several paths and return the best
   * @param st_paths vector of paths
   * @param path The computed path (optimized solution).
   * @return OK if planning succeeds; error otherwise.
   */
  Status optimizePaths(std::vector<std::vector<STCell>> st_paths,
                       std::vector<STConfig> &path);

private:
  MPCConfig *mpc_config_m;
  Vehicle *vehicle_m;
  MPC mpc_m;

  /**
   * @brief optimize one st_path and return the resulted s_path and the cost
   * value
   * @param st_paths path
   * @param speed_map define the speed limit associated to each s value
   * @param path The computed path (optimized solution).
   * @param cost cost of exectining the path
   * @return OK if planning succeeds; error otherwise.
   */
  Status optimizePath(std::vector<STCell> st_path,
                      std::map<int, double> speed_map,
                      std::vector<STConfig> &path, double &cost);
  /**
   * @brief verify if the st_path is valid and return True if it is valid
   * st_path
   * @param st_paths path
   * @param speed_map define the speed limit associated to each s value
   * @return True if the path is valid.
   */
  bool evaluatePath(std::vector<STCell> st_path,
                    std::map<int, double> speed_map);
};

} // namespace planning

#endif // ST_OPTIMIZER_H
