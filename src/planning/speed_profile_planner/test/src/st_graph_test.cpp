/**
 * @file st_graph_test.cpp
 * @brief StGraph class test.
 **/

#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "speed_planner_lib/log.h"
#include "speed_planner_lib/obstacle.h"
#include "speed_planner_lib/st_graph.h"

using namespace planning;

class STGraphTest {
public:
  STGraphTest() = default;

  STGraphTest(std::string filename) { Run(); }

  std::vector<Point2D> LoadReferencePath(std::string filename);
  void PrintResult(std::vector<STArea> areas);
  void SaveResult(std::vector<STArea> areas);

  int Run();

private:
  std::string filename_;
};

std::vector<Point2D> STGraphTest::LoadReferencePath(std::string filename) {
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

void STGraphTest::PrintResult(std::vector<STArea> areas) {
  DEBUG_MSG("Print Result ...");
  for (int i = 0; i < areas.size(); i++) {
    DEBUG_MSG(areas[i].cut_in.s
              << ", " << areas[i].cut_in.t << ", " << areas[i].cut_out.s << ", "
              << areas[i].cut_out.t << ", " << areas[i].upper_bound << ", "
              << areas[i].lower_bound);
  }
}

void STGraphTest::SaveResult(std::vector<STArea> areas) {
  DEBUG_MSG("Save Result ...");
  std::ofstream output;
  output.open("../unitTesting/st_graph_areas.csv");
  for (int i = 0; i < areas.size(); i++) {
    output << areas[i].cut_in.s << ", " << areas[i].cut_in.t << ", "
           << areas[i].cut_out.s << ", " << areas[i].cut_out.t << ", "
           << areas[i].upper_bound << ", " << areas[i].lower_bound << std::endl;
  }
  output.close();
}

int STGraphTest::Run() {
  DEBUG_MSG("");
  DEBUG_MSG("STGraph class Test");

  std::string reference_filepath = "../unitTesting/target_path.csv";
  std::string obstacle_filepath = "../unitTesting/obstacle_path.csv";

  // Lincoln MKZ
  const double vehicle_width = 2.11;
  const double vehicle_length = 4.93;

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

  PrintResult(areas);
  SaveResult(areas);

  DEBUG_MSG("");

  return 0;
}

int main(int argc, char *argv[]) {
  /** test: STGraph class **/
  DEBUG_MSG("***** STGraph Test: *****");

  STGraphTest st_graph_test;
  st_graph_test.Run();

  DEBUG_MSG("The end of main.");
  return 0;
}
