#include <XmlRpcCpp.h>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <vector>

// ros
#include <casper_auto_msgs/MapBasepath.h>
#include <casper_auto_msgs/MapBoundary.h>
#include <casper_auto_msgs/MapCrosswalk.h>
#include <casper_auto_msgs/MapLane.h>
#include <casper_auto_msgs/MapObject.h>
#include <casper_auto_msgs/MapStopLine.h>
#include <casper_auto_msgs/MapTrafficLight.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <casper_auto_msgs/CenterLanes.h>
#include <casper_auto_msgs/VehicleState.h>

#include <odr_map.h>

class OdrMapPathFinder {
public:
  OdrMapPathFinder(ros::NodeHandle nh, std::shared_ptr<OdrMap> map_parser);

private:
  void initalize();

  int getNearestPathIdx(std::vector<double> &query_loc);

  std::vector<int> reconstructPath(int curr,
                                   std::unordered_map<int, int> &parent);

  std::vector<geometry_msgs::Point>
  pathSearch(std::vector<double> &start_location,
             std::vector<double> &goal_localtion);

  std::vector<geometry_msgs::Point>
  convertGeometryPoints(std::vector<geometry_msgs::Point32> &points);

  visualization_msgs::Marker
  convertToPathMarkerMsg(std::vector<geometry_msgs::Point> &points,
                         int marker_id);

  visualization_msgs::Marker
  convertToPathIDMarkerMsg(geometry_msgs::Point &point, int marker_id);

  nav_msgs::Path convertToNavPathMsg(std::vector<geometry_msgs::Point> &points);

  void timerCallback(const ros::TimerEvent &e);

private:
  ros::NodeHandle nh_;

  // ros timer
  ros::Timer timer_;

  // ROS Publishers
  ros::Publisher lanes_marker_pub_;
  ros::Publisher lanes_id_marker_pub_;
  ros::Publisher reference_lanes_pub_;

  // msg
  visualization_msgs::MarkerArrayPtr lanes_msg_;
  visualization_msgs::MarkerArrayPtr lanes_id_msg_;
  casper_auto_msgs::CenterLanesPtr reference_lanes_msg_;

  // map_parser
  std::shared_ptr<OdrMap> map_parser_;

  // map objects
  std::vector<casper_auto_msgs::MapBasepath> map_paths_;

  //
  std::vector<int> lane_indexes_;
  std::vector<std::vector<geometry_msgs::Point>> interested_lanes_;
};

OdrMapPathFinder::OdrMapPathFinder(ros::NodeHandle nh,
                                   std::shared_ptr<OdrMap> map_parser)
    : nh_(nh), map_parser_(map_parser) {
  initalize();
}

void OdrMapPathFinder::initalize() {
  ////////////////////////////////////////////////
  // Load Params
  ////////////////////////////////////////////////
  std::string scenario;
  nh_.param<std::string>("/scenario/type_scenario", scenario, "cross-traffic");

  XmlRpc::XmlRpcValue indexes;
  nh_.getParam("/scenario/lanes", indexes);
  if (indexes.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < indexes.size(); i++) {
      lane_indexes_.push_back((int)(indexes[i]));
      ROS_INFO("Lane index = %d.", int(indexes[i]));
    }
  }

  std::vector<std::string> keys(lane_indexes_.size() * 2);
  for (int i = 0; i < lane_indexes_.size(); i++) {
    keys[i * 2] =
        "/scenario/start_point_lane" + std::to_string(lane_indexes_[i]);
    keys[i * 2 + 1] =
        "/scenario/target_point_lane" + std::to_string(lane_indexes_[i]);
    // ROS_INFO(keys[i * 2].c_str());
    // ROS_INFO(keys[i * 2 + 1].c_str());
  }

  std::vector<std::vector<double>> endpoints(keys.size());
  for (int k = 0; k < keys.size(); k++) {
    XmlRpc::XmlRpcValue point;
    nh_.getParam(keys[k], point);
    if (point.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      std::vector<double> p(point.size());
      for (int i = 0; i < point.size(); i++) {
        p[i] = (double)(point[i]);
      }
      endpoints[k] = p;
      ROS_INFO("Point x = %f, y = %f", p[0], p[1]);
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  // Initalize Publishers
  //////////////////////////////////////////////////////////////////////////////
  lanes_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/casper_auto/odr_map/reference_lanes_marker", 1);
  lanes_id_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/casper_auto/odr_map/reference_lanes_id_marker", 1);
  reference_lanes_pub_ = nh_.advertise<casper_auto_msgs::CenterLanes>(
      "/casper_auto/odr_map/reference_lanes", 1);

  // marker ptr
  lanes_msg_.reset(new visualization_msgs::MarkerArray);
  lanes_id_msg_.reset(new visualization_msgs::MarkerArray);
  reference_lanes_msg_.reset(new casper_auto_msgs::CenterLanes);

  //////////////////////////////////////////////////////////////////////////
  // Get All Map Info
  //////////////////////////////////////////////////////////////////////////
  casper_auto_msgs::MapObject map_obj = map_parser_->GetAllMapObjects();
  map_paths_ = map_obj.basepaths;

  //////////////////////////////////////////////////////////////////////////
  // PathFinding
  //////////////////////////////////////////////////////////////////////////
  interested_lanes_.resize(lane_indexes_.size());
  for (int i = 0; i < lane_indexes_.size(); i++) {
    std::vector<geometry_msgs::Point> path =
        pathSearch(endpoints[i * 2], endpoints[i * 2 + 1]);
    interested_lanes_[i] = path;
  }

  //////////////////////////////////////////////////////////////////////////
  // MarkerArray msg
  //////////////////////////////////////////////////////////////////////////
  // interested lanes/paths
  lanes_msg_->markers.resize(interested_lanes_.size());
  for (int i = 0; i < interested_lanes_.size(); i++) {
    lanes_msg_->markers[i] =
        convertToPathMarkerMsg(interested_lanes_[i], lane_indexes_[i]);
  }

  // interested lane/path ids
  lanes_id_msg_->markers.resize(interested_lanes_.size());
  for (int i = 0; i < interested_lanes_.size(); i++) {
    if (interested_lanes_[i].empty())
      continue;
    int index = interested_lanes_[i].size() * 0.25;
    geometry_msgs::Point point = interested_lanes_[i][index];
    lanes_id_msg_->markers[i] =
        convertToPathIDMarkerMsg(point, lane_indexes_[i]);
  }

  // reference_lanes
  reference_lanes_msg_->header.frame_id = "map";
  reference_lanes_msg_->header.stamp = ros::Time::now();
  reference_lanes_msg_->ids = lane_indexes_;
  reference_lanes_msg_->lanes_width =
      std::vector<float>(lane_indexes_.size(), 3.5);
  reference_lanes_msg_->center_lines.resize(lane_indexes_.size());
  for (int i = 0; i < lane_indexes_.size(); i++) {
    reference_lanes_msg_->center_lines[i] =
        convertToNavPathMsg(interested_lanes_[i]);
  }

  // createTimer
  timer_ = nh_.createTimer(ros::Duration(0.1), &OdrMapPathFinder::timerCallback,
                           this);
}

int OdrMapPathFinder::getNearestPathIdx(std::vector<double> &query_loc) {
  int idx = -1;

  double t = 3;
  double tmp_t, r, prev_r, x1, y1, x2, y2, a, b, c, x, gap;
  prev_r = -1;

  double query_x = query_loc[0];
  double query_y = query_loc[1];

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

std::vector<int>
OdrMapPathFinder::reconstructPath(int curr,
                                  std::unordered_map<int, int> &parent) {
  std::vector<int> found_path;
  found_path.push_back(curr);
  while (parent.count(curr)) {
    found_path.push_back(parent[curr]);
    curr = parent[curr];
  }
  reverse(found_path.begin(), found_path.end());
  for (int id : found_path) {
    std::cout << id << ", ";
  }
  std::cout << std::endl;
  return found_path;
}

std::vector<geometry_msgs::Point>
OdrMapPathFinder::pathSearch(std::vector<double> &start_location,
                             std::vector<double> &goal_localtion) {
  int start_index = getNearestPathIdx(start_location);
  int goal_index = getNearestPathIdx(goal_localtion);

  // bfs
  std::vector<int> found_path;

  std::queue<int> q;
  q.push(start_index);

  std::unordered_set<int> visited;
  std::unordered_map<int, int> parent;

  while (!q.empty()) {
    auto &t = q.front();
    q.pop();

    if (t == goal_index) {
      found_path = reconstructPath(goal_index, parent);
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
  for (auto &id : found_path) {
    std::vector<geometry_msgs::Point> points =
        convertGeometryPoints(map_paths_[id].points);
    path_points.insert(path_points.end(), points.begin(), points.end());
  }

  return path_points;
}

std::vector<geometry_msgs::Point> OdrMapPathFinder::convertGeometryPoints(
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

visualization_msgs::Marker OdrMapPathFinder::convertToPathMarkerMsg(
    std::vector<geometry_msgs::Point> &points, int marker_id) {
  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = "map";
  // path_marker.header.stamp = ros::Time::now();
  path_marker.ns = "path";
  path_marker.id = marker_id;
  path_marker.type =
      visualization_msgs::Marker::LINE_STRIP; // or POINTS, LINE_STRIP
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.pose.orientation.w = 1.0;

  // Size
  path_marker.scale.x = 1.0;

  // Color
  path_marker.color.r = 1.0;
  path_marker.color.g = 1.0;
  path_marker.color.b = 0.0;
  path_marker.color.a = 0.2;

  path_marker.points = points;

  return path_marker;
}

visualization_msgs::Marker
OdrMapPathFinder::convertToPathIDMarkerMsg(geometry_msgs::Point &point,
                                           int marker_id) {
  visualization_msgs::Marker path_id_marker;
  path_id_marker.header.frame_id = "map";
  // path_id_marker.header.stamp = ros::Time::now();
  path_id_marker.ns = "path_id";
  path_id_marker.id = marker_id;
  path_id_marker.type =
      visualization_msgs::Marker::TEXT_VIEW_FACING; // or POINTS, LINE_STRIP
  path_id_marker.action = visualization_msgs::Marker::ADD;
  path_id_marker.pose.position = point;
  path_id_marker.pose.orientation.w = 1.0;

  // Size
  path_id_marker.scale.z = 5.0;

  // Color
  path_id_marker.color.r = 1.0;
  path_id_marker.color.g = 1.0;
  path_id_marker.color.b = 0.0;
  path_id_marker.color.a = 1.0;

  path_id_marker.text = std::to_string(marker_id);

  return path_id_marker;
}

nav_msgs::Path OdrMapPathFinder::convertToNavPathMsg(
    std::vector<geometry_msgs::Point> &points) {
  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  path.poses.resize(points.size());
  for (int i = 0; i < points.size(); i++) {

    path.poses[i].header.frame_id = "map";
    path.poses[i].header.stamp = ros::Time::now();
    path.poses[i].pose.position = points[i];

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

    path.poses[i].pose.orientation = tf2::toMsg(quat_tf);
  }

  return path;
}

void OdrMapPathFinder::timerCallback(const ros::TimerEvent &e) {
  // interested lanes/paths
  lanes_marker_pub_.publish(lanes_msg_);
  // interested lane/path ids
  lanes_id_marker_pub_.publish(lanes_id_msg_);
  // reference_lanes
  reference_lanes_pub_.publish(reference_lanes_msg_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odr_map_path_finder");
  ros::NodeHandle nh("~");

  ////////////////////////////////////////////////////////////////////////////
  // ROS Param Server
  ////////////////////////////////////////////////////////////////////////////
  double map_step;

  nh.param<double>("map_step", map_step, 1.0);

  ////////////////////////////////////////////////////////////////////////////
  // Wait for opendrive map msg
  ////////////////////////////////////////////////////////////////////////////
  boost::shared_ptr<std_msgs::String const> shared_map_msg =
      ros::topic::waitForMessage<std_msgs::String>(
          "/carla/map", nh);
  std::string xml_str = shared_map_msg->data;

  /////////////////////////////////////////////////////////////////
  // Data type conversion and value checking
  /////////////////////////////////////////////////////////////////
  std::shared_ptr<OdrMap> map_parser =
      std::make_shared<OdrMap>(xml_str, map_step);

  ////////////////////////////////////////////////////////////////////////////
  // Map Loader instance
  ////////////////////////////////////////////////////////////////////////////
  OdrMapPathFinder path_finder(nh, map_parser);

  ////////////////////////////////////////////////////////////////////////////
  // ROS Spin
  ////////////////////////////////////////////////////////////////////////////
  ros::spin();

  return 0;
}
