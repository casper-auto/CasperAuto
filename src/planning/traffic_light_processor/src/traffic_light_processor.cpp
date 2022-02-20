#include "traffic_light_processor.h"

#include <iostream>

double distance2D(geometry_msgs::Point a, geometry_msgs::Point b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return sqrt(dx * dx + dy * dy);
}

TrafficLightProcessor::TrafficLightProcessor(ros::NodeHandle nh,
                                             std::shared_ptr<OdrMap> map_parser)
    : nh_(nh), map_parser_(map_parser) {
  initalize();
}

void TrafficLightProcessor::initalize() {
  //////////////////////////////////////////////////////////////////////////////
  // Initalize Subscribers & Publishers
  //////////////////////////////////////////////////////////////////////////////
  current_pose_sub_ = nh_.subscribe(
      "/current_pose", 1, &TrafficLightProcessor::currentPoseCallback, this);
  selected_path_sub_ = nh_.subscribe(
      "/selected_path", 1, &TrafficLightProcessor::selectedPathCallback, this);
  traffic_light_state_sub_ =
      nh_.subscribe("/traffic_light_state", 1,
                    &TrafficLightProcessor::trafficLightStateCallback, this);

  stop_point_pub_ =
      nh_.advertise<geometry_msgs::PointStamped>("/stop_point", 10);
  stop_line_marker_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/stop_line_marker", 10);
  stop_line_activated_pub_ =
      nh_.advertise<std_msgs::Bool>("/stop_line_activated", 10);
  multi_lane_flag_pub_ = nh_.advertise<std_msgs::Bool>("/multi_lane_flag", 10);

  //////////////////////////////////////////////////////////////////////////
  // Get All Map Info
  //////////////////////////////////////////////////////////////////////////
  casper_auto_msgs::MapObject map_obj = map_parser_->GetAllMapObjects();
  map_paths_ = map_obj.basepaths;
  map_lane_boundaries_ = map_obj.lanes;

  current_pose_initialized_ = false;
  multi_lane_flag_ = false;

  // createTimer
  timer_ = nh_.createTimer(ros::Duration(0.1),
                           &TrafficLightProcessor::timerCallback, this);
}

void TrafficLightProcessor::timerCallback(const ros::TimerEvent &e) {
  if (current_pose_initialized_) {
    int current_path_index = getNearestPathIdx(current_position_);
    casper_auto_msgs::MapBasepath current_path = map_paths_[current_path_index];
    auto next_idxes = current_path.next_idxes;

    if (!current_path.left_idxes.empty() || !current_path.right_idxes.empty()) {
      multi_lane_flag_ = true;
    }
    else {
      multi_lane_flag_ = false;
    }
    std_msgs::Bool flag_msg;
    flag_msg.data = multi_lane_flag_;
    multi_lane_flag_pub_.publish(flag_msg);

    // check if traffic light activated
    bool traffic_light_activated = true;
    geometry_msgs::Pose stop_pose;
    if(map_parser_->isInJunction(current_position_.x, current_position_.y, current_position_.z)) {
      traffic_light_activated = false;
    }
    else {
      // find stop_point from selected_path which is one first falls into junction
      for (int i = 0; i < selected_path_.size(); i++) {
        double x = selected_path_[i].position.x;
        double y = selected_path_[i].position.y;
        double z = selected_path_[i].position.z;
        if (map_parser_->isInJunction(x, y, z)) {
          int stop_index = i;
          stop_pose = selected_path_[stop_index];
          break;
        }
      }
    }

    geometry_msgs::PointStamped stop_point_msg;
    stop_point_msg.header.frame_id = "map";
    stop_point_msg.header.stamp = ros::Time();

    // marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    // marker.header.stamp = ros::Time();
    marker.ns = "stop_line";
    marker.id = 0;

    // stop_line_activated
    std_msgs::Bool stop_line_signal;
    stop_line_signal.data = false;

    if (traffic_light_activated && traffic_light_state_ == 1) {
      found_stop_line_ = true;
      stop_line_signal.data = true;
      stop_point_.x = stop_pose.position.x;
      stop_point_.y = stop_pose.position.y;
      stop_point_.z = stop_pose.position.z;
      stop_point_msg.point = stop_point_;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = stop_pose;
      marker.pose.position.z = 2.0;
      marker.scale.x = 5;
      marker.scale.y = 1;
      marker.scale.z = 4;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    } else {
      found_stop_line_ = false;
      stop_point_msg.point.z = -100;
      marker.action = visualization_msgs::Marker::DELETE;
    }
    stop_point_pub_.publish(stop_point_msg);
    stop_line_marker_pub_.publish(marker);
    stop_line_activated_pub_.publish(stop_line_signal);
  }
}

void TrafficLightProcessor::currentPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  // ROS_INFO("current pose got ...");
  current_position_ = msg->pose.position;
  current_pose_initialized_ = true;
}

void TrafficLightProcessor::selectedPathCallback(
    const nav_msgs::Path::ConstPtr &msg) {
  // ROS_INFO("selected_path got ...");
  selected_path_.resize(0);
  for (int i = 0; i < msg->poses.size(); i++) {
    selected_path_.push_back(msg->poses[i].pose);
  }
}

void TrafficLightProcessor::trafficLightStateCallback(
    const std_msgs::Int32::ConstPtr &msg) {
  // ROS_INFO("light state got ...");
  traffic_light_state_ = msg->data;
  // ROS_INFO("%d", traffic_light_state_);
}

int TrafficLightProcessor::getNearestPathIdx(
    geometry_msgs::Point &query_point) {
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

std::vector<int> TrafficLightProcessor::getNearestLaneIdx(
    geometry_msgs::Point &query_point) {
  int idx1 = -1, idx2 = -1; // index of lane & sub index of line in the lane
  double dist = 10000;
  double tmp_t, r, prev_r, x1, y1, x2, y2, a, b, c, x, gap;
  prev_r = -1;

  double query_x = query_point.x;
  double query_y = query_point.y;

  for (int i = 0; i < map_lane_boundaries_.size(); i++) {
    const casper_auto_msgs::MapLane &lane_boundary = map_lane_boundaries_[i];

    std::vector<casper_auto_msgs::LaneInformation> lane_info_array = lane_boundary.lane_information;

    for(int k = 0; k < lane_info_array.size(); k++) {
      auto lane_info = lane_info_array[k];
      for (int j = 0; j < lane_info.points.size() - 1; j++) {
        x1 = lane_info.points[j].x;
        y1 = lane_info.points[j].y;
        x2 = lane_info.points[j + 1].x;
        y2 = lane_info.points[j + 1].y;
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
          if (tmp_t < dist) {
            idx1 = i;
            idx2 = k;
            dist = tmp_t;
          }
        } else if (r <= 0) {
          tmp_t = sqrt((query_x - x1) * (query_x - x1) +
                       (query_y - y1) * (query_y - y1));
          if (tmp_t < dist) {
           idx1 = i;
           idx2 = k;
           dist = tmp_t;
          }
        }
        prev_r = r;
      }
    }
  }

  return {idx1, idx2};
}
