/**
 * @file speed_planner.h
 * @brief The class of SpeedPlanner.
 **/

#ifndef SPEED_PLANNER_H
#define SPEED_PLANNER_H

#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <chrono> // get current timestamp in milliseconds
#include <cmath>  // std::hypot square root since c++ 11
#include <ctime>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "speed_planner_lib/constants.h"
#include "speed_planner_lib/frenet_frame.h"
#include "speed_planner_lib/log.h"
#include "speed_planner_lib/obstacle.h"
#include "speed_planner_lib/st_cell_planner.h"
#include "speed_planner_lib/st_graph.h"
#include "speed_planner_lib/st_optimizer.h"
#include "speed_planner_lib/status.h"
#include "speed_planner_lib/third_party/spline.h"
#include "speed_planner_lib/vehicle.h"

namespace pt = boost::property_tree;
/**
 *
 * \enum    ScenarioID
 *
 * \brief   Values that represent scenario id.
 **/
enum ScenarioID {
  NOT_DEFINED,
  ENTER_ROAD,
  EXIT_ROAD,
  LEFT_TURN,
  RIGHT_TURN,
  CROSS_WALK
};

/**
 *
 * \enum    TrafficLightStatus
 *
 * \brief   Values that represent the status of traffic light.
 **/
enum TrafficLightStatus { NA, RED, YELLOW, GREEN };

/**
 *
 * \enum    StopSignStatus
 *
 * \brief   Values that represent if ego car has stoped for stop sign or not.
 **/
enum StopSignStatus { HANDELED, NOT_HANDELED };

namespace planning {

/**
 * @class SpeedPlanner
 * @brief Implementation of speed planning for a desired path.
 */
class SpeedPlanner {
public:
  /**
   * @brief Constructor
   */
  SpeedPlanner();

  /**
   * @brief Constructor
   */
  SpeedPlanner(MPCConfig *mpc_config, bool left_hand = false,
               bool low_speed_cut = false, double low_speed_thresh = 0.5,
               int velocity_offset = 0, double car_following_distance = 10,
               double oncoming_car_distance = 15, double stop_line_offset = 0.0,
               double release_time = 3.0, double crosswalk_span = 2.0,
               double crosswalk_shift = 0.0, int viz_mode = 0);

  /**
   * @brief Destructor
   */
  ~SpeedPlanner();

  /**
   * @brief Manually reload mpc optimizer config.
   * @param mapfolder
   * @return OK if reloading optimizer config was successfull; error otherwise.
   */
  Status ReloadOptimizerConfig(MPCConfig *mpc_config);

  /**
   * @brief Manually pick / change scenario by a local map file.
   * @param mapfile
   * @return OK if loading map was successfull; error otherwise.
   */
  Status InitScenario(ScenarioID ID);

  /**
   * @brief
   * @param status it can be set to (
   * RED
   * YEALLOW
   * GREEN )
   * @return OK if setting the status was successfull; error otherwise.
   */
  Status SetTrafficLightStatus(TrafficLightStatus status);

  /**
   * @brief This method will populate the trajectory with velocity, relative
   * time. Before planning here, reference_trajectory is only populated with a
   * series of path points (x, y) which comes from the path planning module.
   * @param obstacles All obstacles from perception and prediction.
   * @param reference_trajectory The computed trajectory with the speed profile.
   *      - the first element of reference_trajectory shall represent current
   * configiguration of the ego car (x, y, v).
   *      - reference_trajectory shall represent at least 100m of path,
   * otherwise, the last point will be consider as a point with v = 0 (stop
   * point).
   *      - the distance between the points shall be \f$ 0.5m (+/-0.05m) \f$.
   * @return OK if planning succeeds; error otherwise.
   */
  Status Plan(const std::vector<Obstacle> &perception_obstacles,
              std::vector<TrajectoryPoint> &reference_trajectory);

  /**
   * @brief Set speed limit.
   * @param max_speed maximum speed in \f$ m/s \f$
   * @return OK if loading map was successfull; error otherwise.
   */
  Status SetSpeedLimit(double max_speed);

  /**
   * @brief Set acceleration limits.
   * @param min_acc minimum acceleration (Brake) in \f$ m/s^2 \f$ [negative
   * value]
   * @param max_acc maximum acceleration (Throttle) in \f$ m/s^2 \f$ [positive
   * value]
   * @return OK if loading map was successfull; error otherwise.
   */
  Status SetAccelerationLimits(double min_acc, double max_acc);

  /**
   * @brief Set jerk limits.
   * @param min_jerk minimum jerk (Brake) in \f$ m/s^3 \f$ [negative value]
   * @param max_jerk maximum jerk (Throttle) in \f$ m/s^3 \f$ [positive value]
   * @return OK if loading map was successfull; error otherwise.
   */
  Status SetJerkLimits(double min_jerk, double max_jerk);

  /**
   * @brief Stop sign setter.
   */
  Status SetStopSign(const std::vector<std::vector<double>> &stop_line);

  /**
   * @brief Stop sign resetter.
   */
  Status ResetStopSign();

  /**
   * @brief Crosswalk setter.
   */
  Status SetCrossWalk(const std::vector<std::vector<double>> &cross_walk);

  /**
   * @brief Return next velocity commands in the solution puffer.
   * @return velocity command.
   * @note It's intended to be used by speed_planner developpers. Please don't
   * use this command.
   */
  double NextVelocityCommand();

  //  /**
  //   * @brief Set acceleration and brake latency to be considered during
  //   planning.
  //   * @param latency Brake and Acceleration latency in \f$ sec \f$
  //   * @return OK if loading map was successfull; error otherwise.
  //   */
  //  Status SetActuatorsLatency(double latency);

  /**
   * @brief Ego path getter.
   */
  const std::vector<Point2D> GetEgoPath() const { return ego_path_; }

  /**
   * @brief Ego trajectory getter.
   */
  const std::vector<TrajectoryPoint> GetEgoTraj() const { return ego_traj_; }

  /**
   * @brief Target path getter.
   */
  const std::vector<Point2D> GetTargetPath() const { return target_path_; }

  /**
   * @brief Stop sign getter.
   */
  std::vector<std::vector<Point2D>> GetStopSigns() const { return stop_signs_; }

  /**
   * @brief Pick the corespoinding road path line that the given point lies on.
   * @param The given point.
   * @return The path found.
   */
  std::vector<Point2D> GetRoadLineByPoint(Point2D point);

  /**
   * @brief Pick the corespoinding walk path line that the given point lies on.
   * @param The given point.
   * @return The path found.
   */
  std::vector<Point2D> GetWalkLineByPoint(Point2D point);

  /**
   * @brief Reset the stored values for re-initializing the ego car.
   * @return void.
   */
  void Reset() {
    std::fill(stop_activated_.begin(), stop_activated_.end(), false);
    std::fill(stop_passed_.begin(), stop_passed_.end(), false);
    cmd_vels_ = std::vector<double>();
  }

private:
  // ego vehicle state
  Vehicle *ego_vehicle_;

  // Optimizer
  STOptimizer optimizer_;

  // params
  bool left_hand_;
  bool low_speed_cut_;
  double low_speed_thresh_;
  int velocity_offset_;
  double car_following_distance_;
  double oncoming_car_distance_;
  double stop_line_offset_;
  double release_time_;
  double crosswalk_span_;
  double crosswalk_shift_;
  int visualize_;

  TrafficLightStatus tfl_status_;
  StopSignStatus stp_status_;

  std::vector<double> cmd_vels_;

  std::vector<double> act_vel_history_;

  std::vector<double> cmd_vel_history_;

  double prev_acc_;

  // the path is simply from the reference_trajectory
  std::vector<Point2D> ego_path_;

  std::vector<TrajectoryPoint> ego_traj_;

  // target path the ego car wants to merge
  std::vector<Point2D> target_path_;

  std::vector<std::vector<Point2D>> lane_lines_;

  std::vector<std::vector<Point2D>> side_walks_;

  std::vector<std::vector<Point2D>> crosswalks_;

  std::vector<double> crosswalk_spans_;

  // deal with the stop line
  std::vector<std::vector<Point2D>> stop_signs_;
  std::vector<double> stop_release_time_; // the time from start to release
  std::vector<bool> stop_activated_, stop_passed_;
  std::vector<long> stop_activated_time_, stop_passed_time_;

  // deal with the traffic light
  std::vector<std::vector<Point2D>> traffic_lights_;

  std::vector<Point2D> GetPathByPoint(Point2D point,
                                      std::vector<std::vector<Point2D>> paths);

  /**
   * @brief Load the xml file based on the scenario_id.
   * @param scenario_id The id points to the according xml file.
   * @return OK if loading succefully.
   */
  Status ParseLocalMap(std::string filename);

  /**
   * @brief
   * @param
   * @return
   */
  Status ProcessStopSigns(std::vector<Obstacle> &filtered_obstacles);

  /**
   * @brief
   * @param
   * @return
   */
  Status ProcessCrosswalks(std::vector<Obstacle> &filtered_obstacles);

  /**
   * @brief
   * @param
   * @return
   */
  Status ProcessTrafficLights(std::vector<Obstacle> &filtered_obstacles);

  /**
   * @brief
   * @param
   * @return
   */
  Status ProcessOtherCars(const std::vector<Obstacle> &perception_obstacles,
                          std::vector<Obstacle> &filtered_obstacles);

  /**
   * @brief
   * @param
   * @return
   */
  Status ProcessPedestrians(const std::vector<Obstacle> &perception_obstacles,
                            std::vector<Obstacle> &filtered_obstacles,
                            double t_range, double unit_time,
                            std::vector<STArea> &occupied_areas);

  /**
   * @brief
   * @param
   * @return
   */
  Status
  ProcessIntersectionArea(const std::vector<Obstacle> &perception_obstacles,
                          double t_range, double unit_time,
                          std::vector<STArea> &occupied_areas);

  /**
   * @brief Check if the input traffic light is reached.
   * @param tflight A vector of two points representing a line.
   * @return true when current position is the stopline of the traffic light
   * else false
   */
  bool CheckTrafficLight(std::vector<Point2D> tflight);
};

} // namespace planning

#endif // SPEED_PLANNER_H
