/**
 * @file st_cell_planner.h
 * @brief The class of STCellPlanner.
 */

#ifndef ST_CELL_PLANNER_H
#define ST_CELL_PLANNER_H

#include <vector>

#include "speed_planner_lib/log.h"
#include "speed_planner_lib/st_graph.h"
#include "speed_planner_lib/third_party/matplotlibcpp.h"

namespace plt = matplotlibcpp;

namespace planning {

/**
 * @class STCellPlanner
 * @brief Implementation of the core planner in the STGraph. It will generate a
 * several candidate paths in the STGraph for the optimizer.
 */
class STCellPlanner {
public:
  STCellPlanner();

  /**
   * @brief Construct a STCell Planner with an input ST-Graph.
   * @param st_graph: The input ST-Graph to be planned on.
   * @param unit_time: the time resolution in each cell.
   */
  STCellPlanner(STGraph st_graph, double unit_time);

  /**
   * @brief Construct a STCell Planner with given occupied areas on a ST-Graph.
   * @param occupied_areas: Given occupied areas on a ST-Graph
   * @param s_range: Specify the size of the ST-Graph in s axis.
   * @param t_range: Specify the size of the ST-Graph in t axis.
   * @param unit_time: the time resolution in each cell.
   */
  STCellPlanner(std::vector<STArea> occupied_areas, double s_range,
                double t_range, double unit_time);

  /**
   * @brief Search for all viable paths connecting the origin to the right end
   *        using Breadth-First-Search algorithm.
   * @return A vector of STCell series.
   */
  std::vector<std::vector<STCell>> SearchCandidatePlans();

private:
  double s_range_;
  double t_range_;
  double unit_time_;
  std::vector<double> timeline_;
  std::vector<STArea> occupied_areas_;
  std::vector<std::vector<STCell>> occupied_cells_;
  std::vector<std::vector<STCell>> viable_cells_;

  int GetTimeIndex(double time);

  /**
   * @brief Split the time line into a seies of time intervals.
   * @param time_range: The time line to be splitted.
   * @param unit_time: Specify the time interval size.
   * @return A vector of time-points.
   */
  std::vector<double> SplitTimeline(double time_range, double unit_time);

  /**
   * @brief Generate occupied st_cells for each time interval from a given
   * st_graph
   * @return void
   */
  void ComputeOccupiedSTCells();

  //
  /**
   * @brief Generate viable st_cells for each time interval from a given
   * st_graph.
   * @return void
   */
  void ComputeViableSTCells();
};

} // namespace planning

#endif // ST_CELL_PLANNER_H
