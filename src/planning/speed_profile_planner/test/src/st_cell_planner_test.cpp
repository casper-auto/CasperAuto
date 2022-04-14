/**
 * @file st_cell_planner_test.h
 * @brief StCellPlanner class test.
 **/

#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "speed_planner_lib/log.h"
#include "speed_planner_lib/obstacle.h"
#include "speed_planner_lib/st_cell_planner.h"
#include "speed_planner_lib/st_graph.h"

using namespace planning;

class STCellPlannerTest {
public:
  STCellPlannerTest() = default;

  STCellPlannerTest(STGraph st_graph) { Run(); }

  std::vector<Point2D> LoadReferencePath(std::string filename);

  std::vector<std::vector<STCell>> LoadData(std::string filename);
  void PrintResult(std::vector<std::vector<STCell>> candidate_plans);
  void SaveResult(std::vector<std::vector<STCell>> candidate_plans);

  int Run();

private:
  std::string filename_;
};

std::vector<Point2D>
STCellPlannerTest::LoadReferencePath(std::string filename) {
  std::vector<Point2D> ref_path;
  std::ifstream file(filename);
  if (!file.good()) {
    DEBUG_MSG("File not found!");
    return {};
  }
  std::string line, value, temp;
  while (getline(file, line)) {
    std::vector<std::string> row;
    // for split operation
    std::stringstream s(line);
    while (getline(s, value, ' ')) {
      row.push_back(value);
    }
    // construct a point
    double val1, val2;
    std::istringstream iss1(row[0]);
    iss1 >> val1;
    std::istringstream iss2(row[1]);
    iss2 >> val2;
    Point2D point(val1, val2);
    ref_path.push_back(point);
  }
  return ref_path;
}

void STCellPlannerTest::PrintResult(
    std::vector<std::vector<STCell>> candidate_plans) {
  DEBUG_MSG("Print Result ...");
  for (int i = 0; i < candidate_plans.size(); i++) {
    std::vector<STCell> plan = candidate_plans[i];
    DEBUG_MSG("Candidate Plan [" << i + 1 << "]:");
    for (STCell cell : plan) {
      DEBUG_MSG(cell.min_s << ", " << cell.max_s);
    }
    DEBUG_MSG("");
  }
  return;
}

void STCellPlannerTest::SaveResult(
    std::vector<std::vector<STCell>> candidate_plans) {
  DEBUG_MSG("Save Result ...");
  std::ofstream output;
  output.open("../unitTesting/st_cell_plans.csv");
  for (int i = 0; i < candidate_plans.size(); i++) {
    std::vector<STCell> plan = candidate_plans[i];
    output << "Candidate Plan:" << i + 1 << std::endl;
    for (STCell cell : plan) {
      output << cell.min_s << ", " << cell.max_s << std::endl;
    }
    output << std::endl;
  }
  output.close();
}

int STCellPlannerTest::Run() {
  DEBUG_MSG("");
  DEBUG_MSG("STCellPlanner class Test");

  // prepare st_graph
  std::string reference_filepath = "../unitTesting/target_path.csv";
  std::string obstacle_filepath = "../unitTesting/obstacle_path.csv";

  // load data
  std::vector<Point2D> reference_path = LoadReferencePath(reference_filepath);
  std::vector<Point2D> obstacle_path = LoadReferencePath(obstacle_filepath);

  ObstacleType obs_type = CAR;
  double speed = 2.0;

  int init_idx_1 = 50;
  std::vector<TrajectoryPoint> obs_traj_1;
  for (int i = init_idx_1; i < init_idx_1 + 50 && i < obstacle_path.size();
       i++) {
    TrajectoryPoint traj_point;
    traj_point.x = obstacle_path[i].x;
    traj_point.y = obstacle_path[i].y;
    traj_point.v = speed;
    traj_point.v_limit = 10.0;
    obs_traj_1.push_back(traj_point);
  }
  Obstacle obstacle1(obs_type, obs_traj_1);

  int init_idx_2 = 80;
  std::vector<TrajectoryPoint> obs_traj_2;
  for (int i = init_idx_2; i < init_idx_2 + 50 && i < obstacle_path.size();
       i++) {
    TrajectoryPoint traj_point;
    traj_point.x = obstacle_path[i].x;
    traj_point.y = obstacle_path[i].y;
    traj_point.v = speed;
    traj_point.v_limit = 10.0;
    obs_traj_2.push_back(traj_point);
  }
  Obstacle obstacle2(obs_type, obs_traj_2);

  int init_idx_3 = 110;
  std::vector<TrajectoryPoint> obs_traj_3;
  for (int i = init_idx_3; i < init_idx_3 + 50 && i < obstacle_path.size();
       i++) {
    TrajectoryPoint traj_point;
    traj_point.x = obstacle_path[i].x;
    traj_point.y = obstacle_path[i].y;
    traj_point.v = speed;
    traj_point.v_limit = 10.0;
    obs_traj_3.push_back(traj_point);
  }
  Obstacle obstacle3(obs_type, obs_traj_3);

  DEBUG_MSG("Loaded data ...");
  DEBUG_MSG("The number of points is " << reference_path.size());
  if (reference_path.size() < 1) {
    return 1;
  }

  // result
  std::vector<Obstacle> obstacles;
  obstacles.push_back(obstacle1);
  obstacles.push_back(obstacle2);
  obstacles.push_back(obstacle3);

  double s_range = 100;
  double t_range = 20;
  double unit_time = 0.5;

  STGraph st_graph(reference_path, obstacles, s_range, t_range, unit_time);
  std::vector<STArea> areas = st_graph.GetAllOccupied();

  // construct st_cell_planner
  STCellPlanner st_cell_planner(st_graph, unit_time);
  std::vector<std::vector<STCell>> candidate_plans =
      st_cell_planner.SearchCandidatePlans();

  DEBUG_MSG("The number of solutions: " << candidate_plans.size());
  PrintResult(candidate_plans);
  SaveResult(candidate_plans);

  DEBUG_MSG("");

  return 0;
}

int main(int argc, char *argv[]) {
  /** test: st_cell_planner class **/
  DEBUG_MSG("***** STCellPlanner Test: *****");

  STCellPlannerTest st_cell_planner_test;
  st_cell_planner_test.Run();

  DEBUG_MSG("The end of main.");
  return 0;
}
