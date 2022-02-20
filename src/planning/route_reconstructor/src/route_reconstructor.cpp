#include "route_reconstructor.h"

#include <iostream>

double distance2D(geometry_msgs::Point a, geometry_msgs::Point b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return sqrt(dx * dx + dy * dy);
}

bool foundInVector(std::vector<int> vec, int ind) {
  bool found = false;
  for (int i : vec) {
    if (i == ind) {
      found = true;
      break;
    }
  }
  return found;
}

bool foundInVector(std::vector<unsigned int> vec, int ind) {
  bool found = false;
  for (int i : vec) {
    if (i == ind) {
      found = true;
      break;
    }
  }
  return found;
}

double distanceToPath(std::vector<geometry_msgs::Point> &points,
                      geometry_msgs::Point query) {
  double t = 1000;
  double query_x = query.x;
  double query_y = query.y;
  double x, x1, y1, x2, y2, a, b, c, gap, r, tmp_t;
  for (int i = 0; i < points.size() - 1; i++) {
    x1 = points[i].x;
    y1 = points[i].y;
    x2 = points[i + 1].x;
    y2 = points[i + 1].y;
    a = y2 - y1;
    b = x1 - x2;
    c = x2 * y1 - x1 * y2;
    gap = sqrt(a * a + b * b);
    x = (b * b * query_x - a * b * query_y - a * c) / gap / gap; // projection?
    r = (x - x1) / (x2 - x1);

    if (0 < r && r < 1) {
      tmp_t = fabs(a * query_x + b * query_y + c);
      tmp_t /= gap;
      if (tmp_t < t) {
        t = tmp_t;
        // idx = i;
      }
    } else {
      tmp_t = sqrt((query_x - x1) * (query_x - x1) +
                   (query_y - y1) * (query_y - y1));
      if (tmp_t < t) {
        t = tmp_t;
        // idx = i;
      }
    }
  }
  return t;
}

RoutePlanner::RoutePlanner(ros::NodeHandle nh,
                           std::shared_ptr<OdrMap> map_parser,
                           double route_distance, double plan_cycle)
    : nh_(nh), map_parser_(map_parser), route_distance_(route_distance),
      plan_cycle_(plan_cycle) {
  initalize();
}

void RoutePlanner::initalize() {
  //////////////////////////////////////////////////////////////////////////////
  // Initalize Publishers
  //////////////////////////////////////////////////////////////////////////////
  current_pose_sub_ = nh_.subscribe("/casper_auto/localization/current_pose", 1,
                                    &RoutePlanner::currentPoseCallback, this);
  waypoints_sub_ = nh_.subscribe("/carla/ego_vehicle/waypoints", 1,
                                 &RoutePlanner::waypointsCallback, this);

  global_route_pub_ =
      nh_.advertise<nav_msgs::Path>("/casper_auto/planning/global_route", 10);

  //////////////////////////////////////////////////////////////////////////
  // Get All Map Info
  //////////////////////////////////////////////////////////////////////////
  casper_auto_msgs::MapObject map_obj = map_parser_->GetAllMapObjects();
  map_paths_ = map_obj.basepaths;

  current_pose_initialized_ = false;

  // Initial search
  planRoute();

  // createTimer
  timer_ =
      nh_.createTimer(ros::Duration(0.1), &RoutePlanner::timerCallback, this);
}

void RoutePlanner::planRoute() {
  //////////////////////////////////////////////////////////////////////////
  // PathFinding
  //////////////////////////////////////////////////////////////////////////
  if (current_pose_initialized_ && !waypoints_.poses.empty()) {
    target_point_ = getTargetPoint(waypoints_, route_distance_);
    std::vector<geometry_msgs::Point> path_points =
        pathSearch(start_point_, target_point_);

    ROS_INFO("Find a global route plan.");

    //////////////////////////////////////////////////////////////////////////
    // Path msg
    //////////////////////////////////////////////////////////////////////////
    global_route_ = convertToNavPathMsg(path_points);
  }
}

void RoutePlanner::timerCallback(const ros::TimerEvent &e) {
  double current_timestamp = ros::Time::now().toSec();
  if (global_route_.poses.empty() ||
      current_timestamp - prev_timestamp_ > plan_cycle_) {
    planRoute();
    prev_timestamp_ = current_timestamp;
  }

  global_route_.header.frame_id = "map";
  global_route_.header.stamp = ros::Time::now();
  global_route_pub_.publish(global_route_);
}

void RoutePlanner::currentPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  // ROS_INFO("current pose got ...");
  start_point_ = msg->pose.position;
  current_pose_initialized_ = true;
}

void RoutePlanner::waypointsCallback(const nav_msgs::Path::ConstPtr &msg) {
  // ROS_INFO("waypoints got ...");
  waypoints_ = *msg;
}

int RoutePlanner::getClosestIndex(nav_msgs::Path waypoints,
                                  geometry_msgs::Point point) {
  double closest_distance = DBL_MAX;
  int closest_index = 0;
  for (int i = 0; i < waypoints_.poses.size(); i++) {
    double temp = distance2D(waypoints_.poses[i].pose.position, point);
    if (temp < closest_distance) {
      closest_distance = temp;
      closest_index = i;
    }
  }
  return closest_index;
}

int RoutePlanner::getClosestIndex(std::vector<geometry_msgs::Point> points,
                                  geometry_msgs::Point point) {
  double closest_distance = DBL_MAX;
  int closest_index = 0;
  for (int i = 0; i < points.size(); i++) {
    double temp = distance2D(points[i], point);
    if (temp < closest_distance) {
      closest_distance = temp;
      closest_index = i;
    }
  }
  return closest_index;
}

geometry_msgs::Point RoutePlanner::getTargetPoint(nav_msgs::Path waypoints,
                                                  double route_distance) {
  int closest_index = getClosestIndex(waypoints, start_point_);
  double dist = 0;
  for (int i = closest_index + 1; i < waypoints.poses.size(); i++) {
    dist += distance2D(waypoints.poses[i - 1].pose.position,
                       waypoints.poses[i].pose.position);
    if (dist > route_distance) {
      return waypoints.poses[i].pose.position;
    }
  }
  return waypoints.poses.back().pose.position;
}

std::vector<int>
RoutePlanner::getNearestPathIdx(geometry_msgs::Point &query_point) {
  std::vector<int> idxes;

  double tmp_t, r, prev_r, x1, y1, x2, y2, a, b, c, x, gap;
  prev_r = -1;

  double query_x = query_point.x;
  double query_y = query_point.y;

  for (int i = 0; i < map_paths_.size(); i++) {
    const casper_auto_msgs::MapBasepath &path = map_paths_[i];
    double t = 1000;
    for (int j = 0; j < path.points.size() - 1; j++) {
      x1 = path.points[j].x;
      y1 = path.points[j].y;
      x2 = path.points[j + 1].x;
      y2 = path.points[j + 1].y;
      a = y2 - y1;
      b = x1 - x2;
      c = x2 * y1 - x1 * y2;
      gap = sqrt(a * a + b * b);
      x = (b * b * query_x - a * b * query_y - a * c) / gap /
          gap; // projection?
      r = (x - x1) / (x2 - x1);

      if (0 < r && r < 1) {
        tmp_t = fabs(a * query_x + b * query_y + c);
        tmp_t /= gap;
        if (tmp_t < t) {
          t = tmp_t;
          // idx = i;
        }
      } else if (r <= 0) {
        tmp_t = sqrt((query_x - x1) * (query_x - x1) +
                     (query_y - y1) * (query_y - y1));
        if (tmp_t < t) {
          t = tmp_t;
          // idx = i;
        }
      }
      prev_r = r;
    }

    if (t < 0.5) {
      idxes.push_back(path.id.unique_id);
    }
  }

  std::cout << "Query: ";
  for (int i : idxes) {
    std::cout << i << ", ";
  }
  std::cout << std::endl;

  return idxes;
}

std::vector<geometry_msgs::Point>
RoutePlanner::BFSSearch(geometry_msgs::Point &start_point,
                        geometry_msgs::Point &target_point) {
  std::vector<int> start_path_indexes = getNearestPathIdx(start_point);
  std::vector<int> target_path_indexes = getNearestPathIdx(target_point);

  // bfs
  std::vector<int> found_paths;

  std::queue<int> q;

  std::unordered_set<int> visited;
  std::unordered_map<int, int> parent;

  for (int idx : start_path_indexes) {
    q.push(idx);
    visited.insert(idx);
  }

  int steps = 0;

  while (!q.empty()) {
    for (int i = 0; i < q.size(); i++) {
      auto &t = q.front();
      q.pop();

      if (steps > 10)
        break;

      if (foundInVector(target_path_indexes, t)) {
        found_paths = reconstructPath(t, parent);
        break;
      }

      for (auto id : map_paths_[t].next_idxes) {
        if (!visited.count(id)) {
          parent[id] = t;
          q.push(id);
          visited.insert(id);
        }
      }

      for (auto id : map_paths_[t].left_idxes) {
        for (auto iid : map_paths_[id].next_idxes) {
          if (!visited.count(iid)) {
            parent[iid] = t;
            q.push(iid);
            visited.insert(iid);
          }
        }
      }

      for (auto id : map_paths_[t].right_idxes) {
        for (auto iid : map_paths_[id].next_idxes) {
          if (!visited.count(iid)) {
            parent[iid] = t;
            q.push(iid);
            visited.insert(iid);
          }
        }
      }
    }
    steps++;
  }

  if (found_paths.empty()) {
    return {};
  }

  std::vector<int> passing_paths(found_paths.begin() + 1,
                                 found_paths.end() - 1);

  std::vector<geometry_msgs::Point> path_points;

  std::vector<geometry_msgs::Point> points1 =
      convertGeometryPoints(map_paths_[found_paths[0]].points);
  int idx1 = getClosestIndex(points1, start_point);
  path_points.insert(path_points.end(), points1.begin() + idx1, points1.end());

  for (auto &id : passing_paths) {
    std::vector<geometry_msgs::Point> points =
        convertGeometryPoints(map_paths_[id].points);
    path_points.insert(path_points.end(), points.begin(), points.end());
  }

  std::vector<geometry_msgs::Point> points2 =
      convertGeometryPoints(map_paths_[found_paths.back()].points);
  int idx2 = getClosestIndex(points2, target_point);
  path_points.insert(path_points.end(), points2.begin(),
                     points2.begin() + idx2 + 1);

  return path_points;
}

std::vector<geometry_msgs::Point>
RoutePlanner::pathSearch(geometry_msgs::Point &start_point,
                         geometry_msgs::Point &target_point) {
  std::vector<geometry_msgs::Point> path_points;

  int start_index = getClosestIndex(waypoints_, start_point);
  int end_index = getClosestIndex(waypoints_, target_point);

  int prev = 0;
  geometry_msgs::Point prev_point = waypoints_.poses[prev].pose.position;
  std::vector<int> prev_path_indexes = getNearestPathIdx(prev_point);

  path_points.push_back(prev_point);

  for (int curr = 1; curr < waypoints_.poses.size(); curr++) {
    geometry_msgs::Point curr_point = waypoints_.poses[curr].pose.position;
    std::vector<int> curr_path_indexes = getNearestPathIdx(curr_point);

    if (curr_path_indexes.size() != 1)
      continue;

    if (prev_path_indexes.size() == 1 && curr_path_indexes.size() == 1) {

      int prev_path_index = prev_path_indexes[0];
      int curr_path_index = curr_path_indexes[0];

      if (prev_path_index == curr_path_index) {
        std::vector<geometry_msgs::Point> points =
            convertGeometryPoints(map_paths_[curr_path_index].points);
        int idx1 = getClosestIndex(points, prev_point);
        int idx2 = getClosestIndex(points, curr_point);
        path_points.insert(path_points.end(), points.begin() + idx1 + 1,
                           points.begin() + idx2 + 1);
      } else if (prev_path_index != curr_path_index &&
                 foundInVector(map_paths_[prev_path_index].next_idxes,
                               curr_path_index)) {
        std::vector<geometry_msgs::Point> points1 =
            convertGeometryPoints(map_paths_[prev_path_index].points);
        int idx1 = getClosestIndex(points1, prev_point);
        path_points.insert(path_points.end(), points1.begin() + idx1,
                           points1.end());
        std::vector<geometry_msgs::Point> points2 =
            convertGeometryPoints(map_paths_[curr_path_index].points);
        int idx2 = getClosestIndex(points2, curr_point);
        path_points.insert(path_points.end(), points2.begin() + 1,
                           points2.begin() + idx2 + 1);
      } else if (prev_path_index != curr_path_index &&
                 foundInVector(map_paths_[prev_path_index].left_idxes,
                               curr_path_index)) {
        path_points.push_back(curr_point);
      } else if (prev_path_index != curr_path_index &&
                 foundInVector(map_paths_[prev_path_index].right_idxes,
                               curr_path_index)) {
        path_points.push_back(curr_point);
      } else {
        std::cout << "****" << std::endl;
        std::vector<geometry_msgs::Point> points =
            BFSSearch(prev_point, curr_point);
        path_points.insert(path_points.end(), points.begin(), points.end());
        path_points.push_back(curr_point);
      }
    }

    prev = curr;
    prev_point = curr_point;
    prev_path_indexes = curr_path_indexes;
  }

  return path_points;
}

std::vector<geometry_msgs::Point> RoutePlanner::convertGeometryPoints(
    std::vector<geometry_msgs::Point32> &points) {
  std::vector<geometry_msgs::Point> new_points;
  for (auto &p : points) {
    geometry_msgs::Point new_p;
    new_p.x = p.x;
    new_p.y = p.y;
    new_p.z = p.z;
    new_points.push_back(new_p);
  }
  return new_points;
}

std::vector<int>
RoutePlanner::reconstructPath(int curr, std::unordered_map<int, int> &parent) {
  std::vector<int> found_paths;
  found_paths.push_back(curr);
  while (parent.count(curr)) {
    found_paths.push_back(parent[curr]);
    curr = parent[curr];
  }
  reverse(found_paths.begin(), found_paths.end());
  for (int id : found_paths) {
    std::cout << id << ", ";
  }
  std::cout << std::endl;
  return found_paths;
}

nav_msgs::Path
RoutePlanner::convertToNavPathMsg(std::vector<geometry_msgs::Point> &points) {
  int closest_index = getClosestIndex(points, start_point_);
  int size = points.size() - closest_index;

  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  path.poses.resize(size);
  for (int i = closest_index; i < points.size(); i++) {

    path.poses[i - closest_index].header.frame_id = "map";
    // path.poses[i].header.stamp = ros::Time::now();
    path.poses[i - closest_index].pose.position.x = points[i].x;
    path.poses[i - closest_index].pose.position.y = points[i].y;
    path.poses[i - closest_index].pose.position.z = 0.5;

    // compute heading
    double delta_x, delta_y;
    if (i < points.size() - 1) {
      delta_x = points[i + 1].x - points[i].x;
      delta_y = points[i + 1].y - points[i].y;
    } else {
      delta_x = points[i].x - points[i - 1].x;
      delta_y = points[i].y - points[i - 1].y;
    }
    double yaw = atan2(delta_y, delta_x);

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, yaw);

    path.poses[i - closest_index].pose.orientation = tf2::toMsg(quat_tf);
  }

  return path;
}
