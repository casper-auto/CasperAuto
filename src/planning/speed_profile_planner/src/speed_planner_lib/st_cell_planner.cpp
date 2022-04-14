/**
 * @file st_cell_planner.cpp
 * @brief STCellPlanner class definition.
 **/

#include "speed_planner_lib/st_cell_planner.h"

namespace planning {

bool compareSTCell(STCell a, STCell b) {
  if (a.min_s == b.min_s)
    return a.max_s < b.max_s;
  return a.min_s < b.min_s;
}

bool overlap(STCell a, STCell b) {
  if (a.max_s < b.min_s || a.min_s > b.max_s) {
    return false;
  }
  return true;
}

STCellPlanner::STCellPlanner() {}

STCellPlanner::STCellPlanner(STGraph st_graph, double unit_time) {
  s_range_ = st_graph.GetSRange();
  t_range_ = st_graph.GetTRange();
  unit_time_ = std::max(unit_time, 0.1);
  timeline_ = SplitTimeline(t_range_, unit_time_);
  occupied_areas_ = st_graph.GetAllOccupied();
}

STCellPlanner::STCellPlanner(std::vector<STArea> occupied_areas, double s_range,
                             double t_range, double unit_time)
    : occupied_areas_(occupied_areas), s_range_(s_range), t_range_(t_range),
      unit_time_(unit_time) {
  unit_time_ = std::max(unit_time, 0.1);
  timeline_ = SplitTimeline(t_range_, unit_time_);

  DEBUG_MSG("************ Debug: STCellPlanner ***********");
  for (double t : timeline_) {
    DEBUG_MSG("time = " << t);
  }
}

int STCellPlanner::GetTimeIndex(double time) {
  int index = 0;
  while (time > timeline_[index]) {
    index++;
  }
  return --index;
}

// generate occupied st_cells for each time interval from a given st_graph
void STCellPlanner::ComputeOccupiedSTCells() {
  occupied_cells_.resize(timeline_.size());
  for (STArea occupied_area : occupied_areas_) {
    double s_in = occupied_area.cut_in.s;
    double t_in = occupied_area.cut_in.t;
    double s_out = occupied_area.cut_out.s;
    double t_out = occupied_area.cut_out.t;
    double upper_bound = occupied_area.upper_bound;
    double lower_bound = occupied_area.lower_bound;

    int left = 0, right = timeline_.size() - 1;
    while (timeline_[left] < t_in) {
      left++;
    }
    if (left != 0)
      left--;
    while (timeline_[right] > t_out) {
      right--;
    }
    if (right != timeline_.size() - 1)
      right++;

    for (int i = left; i <= right; i++) {
      double s1;
      double s2;
      if (s_out - s_in < 0.001) {
        s1 = s_in;
        s2 = s_in;
      } else {
        s1 = (s_out - s_in) / (t_out - t_in) * (timeline_[i] - t_in) + s_in;
        s2 = (s_out - s_in) / (t_out - t_in) * (timeline_[i + 1] - t_in) + s_in;
      }
      double s_min = std::max(double(0), s1 - lower_bound);
      double s_max = std::min(double(50), s2 + upper_bound);
      STCell st_cell(s_min, s_max, 0);
      if (s_min < s_max)
        occupied_cells_[i].push_back(st_cell);
    }
  }

  DEBUG_MSG("****************************");
  for (int i = 0; i < occupied_cells_.size(); i++) {
    std::vector<STCell> cells = occupied_cells_[i];
    DEBUG_MSG("Occupied cells [" << i + 1 << "]:");
    for (STCell cell : cells) {
      DEBUG_MSG(cell.min_s << ", " << cell.max_s);
    }
  }

  for (int i = 0; i < occupied_cells_.size(); i++) {
    std::vector<STCell> cells = occupied_cells_[i];
    if (cells.empty())
      continue;
    sort(cells.begin(), cells.end(), compareSTCell);
    std::vector<STCell> new_cells;
    new_cells.push_back(cells[0]);
    for (int i = 0; i < cells.size(); i++) {
      if (cells[i].min_s <= new_cells.back().max_s) {
        new_cells.back().max_s =
            std::max(cells[i].max_s, new_cells.back().max_s);
      } else {
        new_cells.push_back(cells[i]);
      }
    }
    occupied_cells_[i] = new_cells;
  }

  DEBUG_MSG("****************************");
  for (int i = 0; i < occupied_cells_.size(); i++) {
    std::vector<STCell> cells = occupied_cells_[i];
    DEBUG_MSG("Occupied cells [" << i + 1 << "]:");
    for (STCell cell : cells) {
      DEBUG_MSG(cell.min_s << ", " << cell.max_s);
    }
  }
}

// generate viable st_cells for each time interval from a given st_graph
void STCellPlanner::ComputeViableSTCells() {
  viable_cells_.resize(timeline_.size());
  for (int i = 0; i < viable_cells_.size(); i++) {
    double prev_border = 0.0;
    std::vector<STCell> cells = occupied_cells_[i];
    sort(cells.begin(), cells.end(), compareSTCell);
    for (STCell cell : cells) {
      viable_cells_[i].push_back(STCell(prev_border, cell.min_s, 0));
      prev_border = cell.max_s;
    }
    viable_cells_[i].push_back(STCell(prev_border, s_range_, 0));
  }

  DEBUG_MSG("****************************");
  for(int i = 0; i < viable_cells_.size(); i++) {
    std::vector<STCell> cells = viable_cells_[i];
    DEBUG_MSG("Viable cells [" << i+1 << "]:");
    for(STCell cell : cells) {
      DEBUG_MSG(cell.min_s << ", " << cell.max_s);
    }
  }
}

// core function
// BFS
std::vector<std::vector<STCell>> STCellPlanner::SearchCandidatePlans() {
  // Search for viable paths connecting viable cells
  std::vector<std::vector<STCell>> candidate_plans;

  ComputeOccupiedSTCells();

  ComputeViableSTCells();

  if (viable_cells_[0].size() > 1)
    viable_cells_[0] = {viable_cells_[0][0]};
  candidate_plans.push_back(viable_cells_[0]);
  for (int i = 1; i < viable_cells_.size(); i++) {
    std::vector<std::vector<STCell>> temp;
    for (std::vector<STCell> plan : candidate_plans) {
      STCell prev_cell = plan.back();

      std::vector<STCell> cells = viable_cells_[i];
      sort(cells.begin(), cells.end(), compareSTCell);
      for (STCell cell : cells) {
        if (overlap(prev_cell, cell)) {
          plan.push_back(cell);
          temp.push_back(plan);
          plan.pop_back();
        }
      }
    }
    candidate_plans = temp;
  }

  std::vector<std::vector<STCell>> new_candidate_plans;
  for (int i = 0; i < candidate_plans.size(); i++) {
    std::vector<STCell> plan = candidate_plans[i];
    bool valid = true;
    for (int j = 0; j < plan.size(); j++) {
      if (plan[j].max_s - plan[j].min_s < 0.001) {
        valid = false;
        break;
      }
    }
    if (valid)
      new_candidate_plans.push_back(plan);
  }

  // DEBUG_MSG("************** Filtered **************");
  // for(int i = 0; i < viable_cells_.size(); i++) {
  //   std::vector<STCell> cells = viable_cells_[i];
  //   DEBUG_MSG("Viable cells [" << i+1 << "]:");
  //   for(STCell cell : cells) {
  //     DEBUG_MSG(cell.min_s << ", " << cell.max_s);
  //   }
  // }

  if (false) {
    plt::figure(1);

    // Clear previous plot
    plt::clf();

    plt::xlim(0, 7);
    plt::ylim(0, 50);

    std::vector<double> xticks;
    for (int i = 0; i <= 7; i++) {
      xticks.push_back(i);
    }

    std::vector<double> yticks;
    for (int i = 0; i <= 50; i += 5) {
      yticks.push_back(i);
    }

    plt::xticks(xticks);
    plt::yticks(yticks);

    for (int i = 0; i < candidate_plans.size(); i++) {
      std::vector<STCell> plan = candidate_plans[i];
      std::vector<double> t, s1, s2;
      for (int j = 0; j < plan.size(); j++) {
        t.push_back(j);
        s1.push_back(plan[j].min_s);
        s2.push_back(plan[j].max_s);
      }

      std::map<std::string, std::string> keywords;
      keywords["alpha"] = "0.4";
      keywords["color"] = "blue";
      // keywords["hatch"] = "-";

      plt::fill_between(t, s1, s2, keywords);
    }

    // Add graph title
    plt::title("ST-Graph");
    plt::xlabel("Time (s)");
    plt::ylabel("Distance along the road (m)");

    // Enable grid.
    plt::grid();
    // Display plot continuously
    plt::pause(0.01);
  }

  return new_candidate_plans;
}

std::vector<double> STCellPlanner::SplitTimeline(double time_range,
                                                 double unit_time) {
  std::vector<double> timeline;
  double curr_time = 0.0;
  while (curr_time < time_range) {
    timeline.push_back(curr_time);
    curr_time += unit_time;
  }
  timeline.push_back(time_range);
  return timeline;
}

} // namespace planning
