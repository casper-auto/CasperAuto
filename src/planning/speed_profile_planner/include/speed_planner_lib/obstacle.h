/**
 * @file obstacle.h
 * @brief The class of Obstacle.
 **/

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "speed_planner_lib/constants.h"
#include "speed_planner_lib/frenet_frame.h"
#include "speed_planner_lib/geometry_utils.h"
#include "speed_planner_lib/log.h"
#include "speed_planner_lib/point2d.h"
#include "speed_planner_lib/trajectory_point.h"

namespace planning {

/**
 * \enum ObstacleType
 * @brief Obstacle type definition.
 */
enum ObstacleType {
  CAR,
  TRUCK,
  BICYCLE,
  PEDESTRIAN,
  TRAFFICLIGHT,
  STOPSIGN,
  CROSSWALK
};

/**
 * @class Obstacle
 *
 * @brief Implementation of obstacle class. It represents cars, trucks,
 * pedestrians and bicycles.
 */

class Obstacle {
public:
  Obstacle() = default;

  /**
   * @brief Constructor.
   * @param obs_type it can be car, pedestrian, bicycle or truck.
   * @param rel_speed The speed along with the ego vehicle's direction in ego
   * vehicle's frame.
   * @param trajectory predicted trajectory of the object. The information needs
   * to be presented with 0.1 sec gap between each 2 points for 10sec.
   * @details Obstacle instance needs to be recreated for each planning
   * iteration. Check Related Page tab for further information.
   */
  Obstacle(ObstacleType obs_type, std::vector<TrajectoryPoint> trajectory,
           bool leading = false);

  ~Obstacle() = default;

  /**
   * @brief Return obstacle type
   * @return
   */
  ObstacleType GetType() const { return obs_type_; }

  /**
   * @brief Return obstacle type
   * @return
   */
  bool IsInTargetLane() const { return in_target_lane_; }

  /**
   * @brief Return obstacle position
   * @return initial position of the obstacle.
   */
  Point2D GetPosition() const { return init_pos_; }

  /**
   * @brief Return obstacle speed
   * @return speed of obstacle.
   */
  double GetSpeed() const { return speed_; }

  /**
   * @brief Return width
   * @return width of obstacle.
   */
  double GetWidth() const { return width_; }

  /**
   * @brief Return length
   * @return length of obstacle.
   */
  double GetLength() const { return length_; }

  /**
   * @deprecated
   * @brief Constat velocity prediction
   * @param time_range time horizon
   * @param unit_time delta time between each 2 points
   * @return predicted trajectory.
   * @details For testing purpose, we do dummy prediction using a reference path
   * and a constant speed.
   */
  std::vector<TrajectoryPoint> PredictTrajectoryOverTime(double time_range,
                                                         double unit_time);

private:
  // obstacle properties
  ObstacleType obs_type_;
  double width_;
  double length_;
  bool in_target_lane_;

  // initial state
  Point2D init_pos_;
  double speed_;
  double init_s_;

  // trajectory prediction (x, y, v, t), s
  std::vector<Point2D> reference_path_;
  std::vector<TrajectoryPoint> predicted_trajectory_;
  Frenet ref_path_frenet_;

  void AdjustSize();

  TrajectoryPoint GetTrajectoryPointAtTime(double time);

  std::vector<double> SplitTimeline(double time_range, double unit_time);
};

} // namespace planning

#endif // OBSTACLE_H
