#ifndef REPLAY_PATH_PLANNER_H
#define REPLAY_PATH_PLANNER_H

#include <vector>
#include <cmath>
#include <cfloat>

using namespace std;

double distance2D(double x1, double y1, double x2, double y2);

template<typename T>
vector<T> slice(vector<T> const &v, int m, int n);

class ReplayPathPlanner {
public:
  ReplayPathPlanner(double lookahead_distance) : m_lookahead_distance(lookahead_distance) {}

  vector<vector<double>> get_replayed_path(vector<double> ego_state, vector<vector<double>> global_route);

  vector<vector<double>> get_interpolated_path(vector<vector<double>> replayed_path, double interp_res = 0.01);

private:
  int m_closest_index;

  double m_closest_distance;

  int m_lookahead_index;

  double m_lookahead_distance;

  vector<vector<double>> m_global_route;

  vector<vector<double>> m_replayed_path;

  vector<vector<double>> m_final_path;

  int get_closest_index(double x, double y, vector<vector<double>> waypoints);

  int get_lookahead_index(double x, double y, vector<vector<double>> waypoints, double lookahead_distance);
};

#endif /* REPLAY_PATH_PLANNER_H */
