/**
 * @file obstacle_test.h
 * @brief obstacle class test
 **/

#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "speed_planner_lib/log.h"
#include "speed_planner_lib/obstacle.h"

using namespace planning;

class ObstacleTest {
public:
  ObstacleTest() = default;

  ObstacleTest(std::string filename) { Run(); }

  std::vector<Point2D> LoadReferencePath(std::string filename);
  void PrintPredictedPath(std::vector<TrajectoryPoint> predicted_traj);
  void SavePredictedPath(std::vector<TrajectoryPoint> predicted_traj);

  int Run();

private:
  std::string filename_;
};

std::vector<Point2D> ObstacleTest::LoadReferencePath(std::string filename) {
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

void ObstacleTest::PrintPredictedPath(
    std::vector<TrajectoryPoint> predicted_traj) {
  DEBUG_MSG("Print Predicted Path ...");
  for (int i = 0; i < predicted_traj.size(); i++) {
    DEBUG_MSG(predicted_traj[i].x << ", " << predicted_traj[i].y);
  }
}

void ObstacleTest::SavePredictedPath(
    std::vector<TrajectoryPoint> predicted_traj) {
  DEBUG_MSG("Save Predicted Path ...");
  std::ofstream output;
  output.open("../unitTesting/predicted_traj.csv");
  for (int i = 0; i < predicted_traj.size(); i++) {
    output << predicted_traj[i].x << ", " << predicted_traj[i].y << ", "
           << predicted_traj[i].relative_time << std::endl;
  }
  output.close();
}

int ObstacleTest::Run() {
  DEBUG_MSG("");
  DEBUG_MSG("Obstacle class Test");

  std::string filename = "../unitTesting/obstacle_path.csv";

  // load data
  std::vector<Point2D> reference_path = LoadReferencePath(filename);
  DEBUG_MSG("Loaded data ...");
  DEBUG_MSG("The number of points is " << reference_path.size());
  if (reference_path.size() < 1) {
    return 1;
  }

  int init_idx = 80;
  Point2D init_pos = reference_path[init_idx];
  double speed = 2.0;

  ObstacleType obs_type = CAR;

  std::vector<TrajectoryPoint> obs_traj;
  for (int i = init_idx; i < init_idx + 50 && i < reference_path.size(); i++) {
    TrajectoryPoint traj_point;
    traj_point.x = reference_path[i].x;
    traj_point.y = reference_path[i].y;
    traj_point.v = speed;
    traj_point.v_limit = 10.0;
    obs_traj.push_back(traj_point);
  }

  Obstacle obstacle(obs_type, obs_traj);

  std::vector<TrajectoryPoint> predicted_traj =
      obstacle.PredictTrajectoryOverTime(20, 0.5);

  // print
  PrintPredictedPath(predicted_traj);
  // output
  SavePredictedPath(predicted_traj);

  DEBUG_MSG("");

  return 0;
}

int main(int argc, char *argv[]) {
  /** test: Obstacle class **/
  DEBUG_MSG("***** Obstacle Test: *****");

  ObstacleTest obstacle_test;
  obstacle_test.Run();

  DEBUG_MSG("The end of main.");
  return 0;
}
