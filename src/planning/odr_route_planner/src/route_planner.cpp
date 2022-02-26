#include "route_planner.h"

#include <iostream>

double distance2D(geometry_msgs::Point a, geometry_msgs::Point b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return sqrt(dx * dx + dy * dy);
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
  // Variables
  //////////////////////////////////////////////////////////////////////////////
  waypoints_defined_ = false;
  goal_defined_ = false;

  //////////////////////////////////////////////////////////////////////////////
  // Initalize Publishers
  //////////////////////////////////////////////////////////////////////////////
  current_pose_sub_ = nh_.subscribe("/casper_auto/current_pose", 1,
                                    &RoutePlanner::currentPoseCallback, this);
  waypoints_sub_ = nh_.subscribe("/casper_auto/waypoints", 1,
                                 &RoutePlanner::waypointsCallback, this);
  goal_sub_ = nh_.subscribe("/casper_auto/goal", 1,
                            &RoutePlanner::goalCallback, this);

  global_route_pub_ =
      nh_.advertise<nav_msgs::Path>("/casper_auto/global_route", 10);

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
  if (current_pose_initialized_) {
    if (waypoints_defined_) {
      target_point_ = getTargetPoint(waypoints_, route_distance_);
    }
    else if (goal_defined_) {
      target_point_ = goal_.position;
    }
    else {
      return;
    }

    std::vector<geometry_msgs::Point> path_points =
        pathSearch(start_point_, target_point_);

    ROS_INFO("Find a global route plan.");

    // Path msg
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
  ROS_INFO("Received a sequnce of waypoints ...");
  waypoints_ = *msg;
  waypoints_defined_ = true;
}

void RoutePlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  ROS_INFO("Received a goal ...");
  goal_ = msg->pose;
  goal_defined_ = true;
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

int RoutePlanner::getNearestPathIdx(geometry_msgs::Point &query_point) {
  int idx = -1;

  double t = 3;
  double tmp_t, r, prev_r, x1, y1, x2, y2, a, b, c, x, gap;
  prev_r = -1;

  double query_x = query_point.x;
  double query_y = query_point.y;

  for (int i = 0; i < map_paths_.size(); i++) {
    const casper_auto_msgs::MapBasepath &path = map_paths_[i];
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
          idx = i;
        }
      } else if (prev_r >= 1 && r <= 0) {
        tmp_t = sqrt((query_x - x1) * (query_x - x1) +
                     (query_y - y1) * (query_y - y1));
        if (tmp_t < t) {
          t = tmp_t;
          idx = i;
        }
      }
      prev_r = r;
    }
  }

  return idx;
}

std::vector<geometry_msgs::Point>
RoutePlanner::pathSearch(geometry_msgs::Point &start_point,
                         geometry_msgs::Point &target_point) {
  int start_index = getNearestPathIdx(start_point);
  int goal_index = getNearestPathIdx(target_point);

  // bfs
  std::vector<int> found_paths;

  std::queue<int> q;
  q.push(start_index);

  std::unordered_set<int> visited;
  std::unordered_map<int, int> parent;

  while (!q.empty()) {
    auto &t = q.front();
    q.pop();

    visited.insert(t);

    if (t == goal_index) {
      found_paths = reconstructPath(goal_index, parent);
      break;
    }

    for (auto id : map_paths_[t].next_idxes) {
      if (!visited.count(id)) {
        parent[id] = t;
        q.push(id);
      }
    }
  }

  std::vector<geometry_msgs::Point> path_points;
  for (auto &id : found_paths) {
    std::vector<geometry_msgs::Point> points =
        convertGeometryPoints(map_paths_[id].points);
    path_points.insert(path_points.end(), points.begin(), points.end());
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
