/**
 * @file st_graph.h
 * @brief The class of ST_Graph.
 */

#ifndef ST_GRAPH_H
#define ST_GRAPH_H

#include <math.h> /* sqrt */
#include <vector>

#include "speed_planner_lib/collision_checker.h"
#include "speed_planner_lib/constants.h"
#include "speed_planner_lib/geometry_utils.h"
#include "speed_planner_lib/log.h"
#include "speed_planner_lib/obstacle.h"
#include "speed_planner_lib/point2d.h"
#include "speed_planner_lib/st_utils.h"
#include "speed_planner_lib/third_party/matplotlibcpp.h"
#include "speed_planner_lib/trajectory_point.h"

namespace plt = matplotlibcpp;

namespace planning {

/**
 * @class STGraph
 *
 * @brief Implemenattion of space-time map.
 */
class STGraph {
public:
  /**
   * @brief Constructor.
   */
  STGraph();

  /**
   * @brief Constructor.
   *        Display the ego car's reference path and the obstacles' predicted
   *        movement / trajectory on a ST-Graph. The size of the graph is
   * spedified by two variables, s_range, t_range.
   * @param reference_path: The ego car's reference path.
   * @param obstacls: A list of obstacles (cars, trucks, and pedestrians)
   * @param s_range: The range in s axis.
   * @param t_range: The range in time axis.
   */
  STGraph(std::vector<Point2D> reference_path, std::vector<Obstacle> obstacles,
          double s_range, double t_range, double unit_time,
          double stop_line_offset = 0.0, double car_following_distance = 10,
          double oncoming_car_distance = 20);

  /**
   * @brief Getter of s_range.
   * @return The value of the range in s axis.
   */
  double GetSRange() const { return s_range_; }

  /**
   * @brief Getter of t_range.
   * @return The value of the range in time axis.
   */
  double GetTRange() const { return t_range_; }

  /**
   * @brief Getter of the list of occupied areas.
   * @return The value of the range in time axis.
   */
  std::vector<STArea> GetAllOccupied() { return occupied_areas_; };

private:
  std::vector<Point2D> reference_path_;
  std::vector<Obstacle> obstacles_;
  double s_range_;
  double t_range_;
  double unit_time_;

  double stop_line_offset_;
  double car_following_distance_;
  double oncoming_car_distance_;

  Frenet ego_path_frenet_;

  std::vector<STArea> occupied_areas_;

  /**
   * @brief Compute the occupied area of the obstacle in ST-Graph
   * @param obstacle
   * @return STArea
   */
  STArea ComputeOccupiedByObstacle(Obstacle obstacle);

  /**
   * @brief Traverse all obstacles and generate the occupied areas in a vector.
   * @param
   * @return
   */
  void ComputeAllOccupied();

  /**
   * @brief Compute the incoming intersection point / collision point of the
   *        obstacle's predicted trajectory and the ego car's path.
   * @param The obstacle's predicted trajectory.
   * @return The STPoint representing the incoming collision point.
   */
  STPoint CutIn(std::vector<TrajectoryPoint> predicted_traj);

  /**
   * @brief Compute the outcoming intersection point / collision point of the
   *        obstacle's predicted trajectory and the ego car's path.
   * @param The obstacle's predicted trajectory.
   * @return The STPoint representing the outcoming collision point.
   */
  STPoint CutOut(std::vector<TrajectoryPoint> predicted_traj);
};

} // namespace planning

#endif // ST_GRAPH_H
