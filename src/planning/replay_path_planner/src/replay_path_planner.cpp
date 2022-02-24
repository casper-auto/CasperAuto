#include "replay_path_planner.h"

#include <iostream>

using namespace std;

double distance2D(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return sqrt(dx * dx + dy * dy);
}

template<typename T>
vector<T> slice(vector<T> const &v, int m, int n) {
  auto first = v.cbegin() + m;
  auto last = v.cbegin() + n + 1;
  vector<T> vec(first, last);
  return vec;
}

vector<vector<double>> ReplayPathPlanner::get_replayed_path(vector<double> current_pose, vector<vector<double>> global_route) {
  double x = current_pose[0];
  double y = current_pose[1];
  m_closest_index = get_closest_index(x, y, global_route);
  m_lookahead_index = get_lookahead_index(x, y, global_route, m_lookahead_distance);
  return slice(global_route, m_closest_index, m_lookahead_index);
}

vector<vector<double>> ReplayPathPlanner::get_interpolated_path(vector<vector<double>> replayed_path, double interp_res) {
  vector<double> wp_distance;
  for(int i = 1; i < replayed_path.size(); i++) {
    wp_distance.push_back(distance2D(replayed_path[i][0], replayed_path[i][1],
                                     replayed_path[i-1][0], replayed_path[i-1][1]));
  }
  wp_distance.push_back(0);

  vector<vector<double>> wp_interp;
  for(int i = 0; i < replayed_path.size()-1; i++) {
    wp_interp.push_back(replayed_path[i]);
    int num_pts_to_interp = floor(wp_distance[i] / interp_res) - 1;
    vector<double> wp_vector = {replayed_path[i+1][0] - replayed_path[i][0],
                                replayed_path[i+1][1] - replayed_path[i][1],
                                replayed_path[i+1][2] - replayed_path[i][2]};
    double vector_len = sqrt(wp_vector[0] * wp_vector[0] + wp_vector[1] * wp_vector[1]);
    vector<double> wp_uvector = {wp_vector[0] / vector_len,
                                 wp_vector[1] / vector_len,
                                 wp_vector[2] / vector_len};
    for(int j = 0; j < num_pts_to_interp; j++) {
      vector<double> next_wp_vector = {interp_res * float(j+1) * wp_uvector[0],
                                       interp_res * float(j+1) * wp_uvector[1],
                                       interp_res * float(j+1) * wp_uvector[2]};
      vector<double> next_addin = {replayed_path[i][0] + next_wp_vector[0],
                                   replayed_path[i][1] + next_wp_vector[1],
                                   replayed_path[i][2] + next_wp_vector[2]};
      wp_interp.push_back(next_addin);
    }
  }
  wp_interp.push_back(replayed_path[replayed_path.size()-1]);
  return wp_interp;
}

int ReplayPathPlanner::get_closest_index(double x, double y, vector<vector<double>> waypoints) {
  double closest_distance = DBL_MAX;
  int closest_index = 0;
  for(int i = 0; i < waypoints.size(); i++) {
      double temp = pow(waypoints[i][0] - x, 2) + pow(waypoints[i][1] - y, 2);
      if(temp < closest_distance) {
        closest_distance = temp;
        closest_index = i;
      }
  }
  m_closest_distance = sqrt(closest_distance);

  return closest_index;
}

int ReplayPathPlanner::get_lookahead_index(double x, double y, vector<vector<double>> waypoints, double lookahead_distance) {
  for(int i = m_closest_index; i < waypoints.size(); i++) {
    double dist = distance2D(x, y, waypoints[i][0], waypoints[i][1]);
    if(dist > lookahead_distance) {
      return i;
    }
  }
  return waypoints.size()-1;
}
