#include "lattice_planner/lattice_planner.h"

using namespace geometry;

namespace planning {

///////////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////////

// Keep the angle within [-pi, pi]
double normalized_angle(double angle) {
  if (angle > M_PI) {
    angle -= 2 * M_PI;
  } else if (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

///////////////////////////////////////////////////////////////////////////////
// class LatticePlanner
///////////////////////////////////////////////////////////////////////////////

LatticePlanner::LatticePlanner(ros::NodeHandle nh) : nh_(nh) {
  initialize();

  ros::Rate rate(10);

  while (ros::ok()) {
    if (!global_route_.empty()) {
      plan();
    }
    //////////////////////////////////////////////////////////////////////////
    // ROS Spin
    //////////////////////////////////////////////////////////////////////////
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("The iteration end.");
  }
}

LatticePlanner::~LatticePlanner() {
  // detroy pointers
}

void LatticePlanner::initialize() {
  //////////////////////////////////////////////////////////////////////////////
  // ROS Param Server
  //////////////////////////////////////////////////////////////////////////////
  nh_.param<std::string>("/global_frame", global_frame_, "map");
  nh_.param<std::string>("/local_frame", local_frame_, "ego_vehicle");
  nh_.param<double>("/lookahead_distance", lookahead_distance_, 20.0);
  nh_.param<double>("/path_distance", path_distance_, 50.0);
  nh_.param<int>("/num_paths", num_paths_, 5);
  nh_.param<double>("/lateral_offset", lateral_offset_, 1.0);
  nh_.param<double>("/longitudinal_offset", longitudinal_offset_, 4.0);
  nh_.param<bool>("/run_parallel", run_parallel_, true);
  nh_.param<bool>("/enable_collision_checker", enable_collision_checker_,
                  false);
  nh_.param<std::vector<double>>("/circle_offsets", circle_offsets_,
                                 {-1.0, 1.0, 3.0});
  nh_.param<std::vector<double>>("/circle_radii", circle_radii_,
                                 {1.5, 1.5, 1.5});
  nh_.param<double>("/path_select_weight", path_select_weight_, 10.0);
  nh_.param<bool>("/enable_velocity_planner", enable_velocity_planner_, false);
  nh_.param<double>("/cruise_speed", cruise_speed_, 5.0);
  nh_.param<double>("/time_step", time_step_, 0.1);
  nh_.param<double>("/accel_max", accel_max_, 1.0);
  nh_.param<double>("/slow_speed", slow_speed_, 1.0);
  nh_.param<bool>("/enable_obstacles_display", enable_obstacles_display_,
                  false);

  lookahead_distance_ = max(lookahead_distance_, G_LOOKAHEAD_MIN);
  lookahead_distance_ = min(lookahead_distance_, G_LOOKAHEAD_MAX);

  path_distance_ = max(path_distance_, lookahead_distance_);

  // ros subscribers
  current_pose_sub_ =
      nh_.subscribe("/current_pose", 1, &LatticePlanner::currentPoseCallback, this);
  current_speed_sub_ =
      nh_.subscribe("/current_speed", 1, &LatticePlanner::currentSpeedCallback, this);
  detected_objects_sub_ = nh_.subscribe(
      "/detected_objects", 1, &LatticePlanner::detectedObjectsCallback, this);
  global_route_sub_ = nh_.subscribe("/global_route", 1,
                                    &LatticePlanner::globalRouteCallback, this);
  stop_line_signal_sub_ = nh_.subscribe(
      "/stop_line_activated", 1, &LatticePlanner::stoplineSignalCallback, this);
  stop_point_sub_ = nh_.subscribe(
      "/stop_point", 1, &LatticePlanner::stopPointCallback, this);

  // ros publishers
  selected_path_pub_ = nh_.advertise<nav_msgs::Path>("/selected_path", 1);
  final_waypoints_pub_ =
      nh_.advertise<casper_auto_msgs::WaypointArray>("/final_waypoints", 1);
  generated_paths_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/generated_paths_marker", 1);
  polygon_array_pub_ = nh_.advertise<jsk_recognition_msgs::PolygonArray>(
      "/obstacle_polygons", 1);

  // initialize instances
  path_optimizer_ = PathOptimizer();
  velocity_planner_ = VelocityPlanner(time_step_, accel_max_, slow_speed_);
  collision_checker_ =
      CollisionChecker(circle_offsets_, circle_radii_, path_select_weight_);

  // initialize driving mode as LaneKeep
  driving_mode_ = DrivingMode::LaneKeep;
  goal_offset_ = lateral_offset_;
  shift_angle_ = M_PI / 2.0;

  follow_lead_vehicle_ = false;
  emergency_stop_ = false;

  tl_detector_initialized_ = false;
  stop_point_initialized_ = false;

  ROS_INFO("Initialization complete.");
}

void LatticePlanner::plan() {
  auto start = std::chrono::high_resolution_clock::now();
  double curr_timestamp = ros::Time::now().toSec();

  // Compute the goal state set from the behavioural planner's computed goal
  // state.
  std::vector<Pose2D> goal_pose_set = getGoalPoseSet();
  // ROS_INFO("The size of goal_pose_set = %d", (int)goal_pose_set.size());

  if (!goal_pose_set.empty()) {

    for (int i = 0; i < goal_pose_set.size(); i++) {
      ROS_INFO("Goal pose %d: %.2f, %.2f, %.2f", i, goal_pose_set[i].position.x,
               goal_pose_set[i].position.y, goal_pose_set[i].yaw);
    }

    // Calculate planned paths in the local frame.
    std::vector<std::vector<Pose2D>> paths = generatePaths(goal_pose_set);

    // Transform those paths back to the global frame.
    std::vector<std::vector<Pose2D>> transformed_paths = transformPaths(paths);

    // Perform collision checking (in local frame and within
    // lookahead_distance)
    std::vector<bool> collision_check_array =
        collision_checker_.collisionCheck(paths, obstacles_);

    // std::cout << "CollisionChecker result:" << std::endl;
    // for(bool r : collision_check_array) {
    //   std::cout << r << ", ";
    // }
    // std::cout << std::endl;

    // Compute the best path.
    int selected_index = collision_checker_.selectBestPathIndex(
        paths, collision_check_array, goal_pose_set[num_paths_ / 2]);
    // std::cout << "selected index = " << selected_index << std::endl;

    // in case there is no valid path
    std::vector<Pose2D> selected_path;
    if (selected_index == -1) {
      selected_path = prev_selected_path_;
    } else {
      selected_path = transformed_paths[selected_index];
    }
    prev_selected_path_ = selected_path;

    // frenet_frame
    Frenet frenet(convertToPointArray(selected_path));

    std::vector<Waypoint2D> vel_profile;

    if (enable_velocity_planner_) {
      // Find the leading vehicle if exists
      // Done in objects Callback

      double open_loop_speed =
          velocity_planner_.getOpenloopSpeed(curr_timestamp - prev_timestamp_);
      current_state_.vel = open_loop_speed;

      // Compute the velocity profile for the path, and compute the waypoints.
      // Use the lead vehicle to inform the velocity profile's dynamic
      // obstacle handling. In this scenario, the only dynamic obstacle is the
      // lead vehicle at index 1.
      double desired_speed = cruise_speed_;

      follow_lead_vehicle_ = false;
      bool too_close = false;
      double distance = 15;
      for (int i = 0; i < detected_objects_.size(); i++) {
        // find the leader vehicle if exists
        Point2D check_p(detected_objects_[i].pose.position.x,
                        detected_objects_[i].pose.position.y);

        Point2D check_p_global =
            localToGlobal(current_state_.position, current_state_.yaw, check_p);

        Point_Frenet p_sd;

        frenet.ToFrenet(check_p_global, p_sd);

        if (p_sd.s > 0 && p_sd.s < distance && abs(p_sd.d) < 1.75) {
          follow_lead_vehicle_ = true;
          distance = distance2D(current_state_.position, check_p_global);

          double vel = detected_objects_[i].twist.linear.x;
          double distance_thresh = 8 + vel * 0.5;

          if (distance < distance_thresh) {
            follow_lead_vehicle_ = false;
            too_close = true;
          }

          double quatx = detected_objects_[i].pose.orientation.x;
          double quaty = detected_objects_[i].pose.orientation.y;
          double quatz = detected_objects_[i].pose.orientation.z;
          double quatw = detected_objects_[i].pose.orientation.w;

          tf::Quaternion q(quatx, quaty, quatz, quatw);
          tf::Matrix3x3 m(q);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);

          leader_state_ = State(check_p.x, check_p.y, yaw, vel);
        }
      }

      bool decelerate_to_stop = too_close || emergency_stop_;

      vel_profile = velocity_planner_.computeVelocityProfile(
          selected_path, desired_speed, current_state_, leader_state_,
          decelerate_to_stop, follow_lead_vehicle_);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    prev_timestamp_ = curr_timestamp;
    ROS_INFO("Elapsed time: %.4f milliseconds.", duration.count() / 1000.0);

    // Publish the final path as an output to other velocity planners
    publishSelectedPath(selected_path);

    // Publish generated paths for visualization
    publishGenratedPaths(transformed_paths, selected_index,
                         collision_check_array);

    // Publish the final waypoints as an output to the controller
    publishFinalWaypoints(vel_profile);
  }

  if (enable_obstacles_display_) {
    // Display obstacles as polygons (in local frame)
    publishObstaclePolygons(obstacles_);
  }
}

/*
goal_pose: Goal pose for the vehicle to reach (local frame)
   format: [x_goal, y_goal, theta], in units [m, m, rad]
*/
std::vector<Pose2D> LatticePlanner::getGoalPoseSet() {
  // get index
  double x = current_state_.position.x;
  double y = current_state_.position.y;

  // pick reference route
  reference_route_ = global_route_;

  // get indexes in the route
  closest_index_ = getClosestIndex(x, y, reference_route_);
  lookahead_index_ =
      getLookaheadIndex(x, y, reference_route_, lookahead_distance_);

  // reach the end of the route
  if (closest_index_ == lookahead_index_)
    return {};

  // goal pose [x, y, theta]
  goal_pose_ = reference_route_[lookahead_index_]; // global frame

  // compute heading
  double heading;
  if (reference_route_.size() >= 2) {
    double delta_x, delta_y;
    if (lookahead_index_ == 0) {
      delta_x = reference_route_[lookahead_index_ + 2].position.x -
                reference_route_[lookahead_index_].position.x;
      delta_y = reference_route_[lookahead_index_ + 2].position.y -
                reference_route_[lookahead_index_].position.y;
    }
    else if (lookahead_index_ < reference_route_.size() - 1) {
      delta_x = reference_route_[lookahead_index_ + 1].position.x -
                reference_route_[lookahead_index_ - 1].position.x;
      delta_y = reference_route_[lookahead_index_ + 1].position.y -
                reference_route_[lookahead_index_ - 1].position.y;
    } else {
      delta_x = reference_route_[lookahead_index_].position.x -
                reference_route_[lookahead_index_ - 2].position.x;
      delta_y = reference_route_[lookahead_index_].position.y -
                reference_route_[lookahead_index_ - 2].position.y;
    }
    heading = atan2(delta_y, delta_x);
  }
  else {
    heading = reference_route_[lookahead_index_].yaw;
  }

  // convert to local frame
  Point2D goal_point_local = globalToLocal(
      current_state_.position, current_state_.yaw, goal_pose_.position);

  double goal_x = goal_point_local.x;
  double goal_y = goal_point_local.y;
  double goal_theta = heading - current_state_.yaw;

  // Keep the goal heading within [-pi, pi]
  goal_theta = normalized_angle(goal_theta);

  std::vector<Pose2D> goal_pose_set;

  for (int i = 0; i < num_paths_; i++) {
    double offset = (i - int(num_paths_ / 2.0)) * goal_offset_;
    double x_offset = offset * cos(goal_theta + shift_angle_);
    double y_offset = offset * sin(goal_theta + shift_angle_);
    Pose2D pose(goal_x + x_offset, goal_y + y_offset, goal_theta);
    goal_pose_set.push_back(pose);
  }

  return goal_pose_set;
}

// Plans the path set using polynomial spiral optimization to
// each of the goal states.
std::vector<std::vector<Pose2D>>
LatticePlanner::generatePaths(std::vector<Pose2D> &goal_pose_set) {
  std::vector<std::vector<Pose2D>> paths;

  if (run_parallel_) {
    // PARALLEL IMPLEMENTATION
    std::vector<std::future<std::vector<std::vector<double>>>> result;
    for (Pose2D goal_pose : goal_pose_set) {
      result.push_back(std::async(
          std::launch::async, &PathOptimizer::optimizeSpiral, path_optimizer_,
          goal_pose.position.x, goal_pose.position.y, goal_pose.yaw));
    }
    // Join threads
    for (int k = 0; k < num_paths_; k++) {
      std::vector<std::vector<double>> spiral = result[k].get();
      std::vector<Pose2D> path = convertToPoseArray(spiral);
      paths.push_back(path);
    }
  } else {
    // NON PARALLEL IMPLEMENTATION
    for (Pose2D goal_pose : goal_pose_set) {
      std::vector<std::vector<double>> spiral = path_optimizer_.optimizeSpiral(
          goal_pose.position.x, goal_pose.position.y, goal_pose.yaw);
      std::vector<Pose2D> path = convertToPoseArray(spiral);
      paths.push_back(path);
    }
  }

  return paths;
}

std::vector<Pose2D>
LatticePlanner::convertToPoseArray(std::vector<std::vector<double>> &spiral) {
  std::vector<Pose2D> path(spiral.size());
  for (int i = 0; i < spiral.size(); i++) {
    path[i] = Pose2D(spiral[i][0], spiral[i][1], spiral[i][2]);
  }
  return path;
}

// Converts the to the global coordinate frame.
// Converts the paths from the local (vehicle) coordinate frame to the
// global coordinate frame.
std::vector<std::vector<Pose2D>>
LatticePlanner::transformPaths(std::vector<std::vector<Pose2D>> &paths) {
  // access current state
  double current_x = current_state_.position.x;
  double current_y = current_state_.position.y;
  double current_theta = current_state_.yaw;

  // coordinate transform
  std::vector<std::vector<Pose2D>> transformed_paths;
  for (int path_id = 0; path_id < paths.size(); path_id++) {
    std::vector<Pose2D> path = paths[path_id];
    std::vector<Pose2D> transformed_path;
    for (int i = 0; i < path.size(); i++) {
      double x = current_x + path[i].position.x * cos(current_theta) -
                 path[i].position.y * sin(current_theta);
      double y = current_y + path[i].position.x * sin(current_theta) +
                 path[i].position.y * cos(current_theta);
      double theta = path[i].yaw + current_theta;
      transformed_path.push_back(Pose2D(x, y, theta));
    }
    transformed_path = extendPath(transformed_path, path_distance_, path_id);
    transformed_paths.push_back(transformed_path);
  }

  return transformed_paths;
}

// In global coordinate frame.
// Extend the generated paths to reach path distance.
std::vector<Pose2D> LatticePlanner::extendPath(std::vector<Pose2D> &path,
                                              double path_distance,
                                              int path_id) {
  // extend from path
  std::vector<Pose2D> extended_path = path;
  double planned_distance = lookahead_distance_;
  for (int i = lookahead_index_; i < reference_route_.size(); i++) {
    if (planned_distance > path_distance)
      break;
    Pose2D pose = reference_route_[i];
    // compute heading
    double delta_x, delta_y;
    if (i < reference_route_.size() - 1) {
      delta_x =
          reference_route_[i + 1].position.x - reference_route_[i].position.x;
      delta_y =
          reference_route_[i + 1].position.y - reference_route_[i].position.y;
      double dist = sqrt(delta_x * delta_x + delta_y * delta_y);
      planned_distance += dist;
    } else {
      delta_x =
          reference_route_[i].position.x - reference_route_[i - 1].position.x;
      delta_y =
          reference_route_[i].position.y - reference_route_[i - 1].position.y;
    }
    double theta = atan2(delta_y, delta_x);
    // offset
    double offset = (path_id - int(num_paths_ / 2.0)) * goal_offset_;
    double x_offset = offset * cos(theta + shift_angle_);
    double y_offset = offset * sin(theta + shift_angle_);
    pose.position.x += x_offset;
    pose.position.y += y_offset;
    extended_path.push_back(pose);
  }

  return extended_path;
}

int LatticePlanner::getClosestIndex(double x, double y,
                                   std::vector<Pose2D> &path) {
  double closest_distance = DBL_MAX;
  int closest_index = 0;
  for (int i = 0; i < path.size(); i++) {
    double temp =
        pow(path[i].position.x - x, 2) + pow(path[i].position.y - y, 2);
    if (temp < closest_distance) {
      closest_distance = temp;
      closest_index = i;
    }
  }
  closest_distance_ = sqrt(closest_distance);
  return closest_index;
}

int LatticePlanner::getLookaheadIndex(double x, double y,
                                     std::vector<Pose2D> &path,
                                     double lookahead_distance) {
  for (int i = closest_index_; i < path.size(); i++) {
    double dist = distance2D(x, y, path[i].position.x, path[i].position.y);
    if (dist > lookahead_distance) {
      return i;
    }
  }
  return path.size() - 1;
}

std::vector<Point2D>
LatticePlanner::convertToPointArray(std::vector<Pose2D> &path) {
  std::vector<Point2D> point_array(path.size());
  for (int i = 0; i < path.size(); i++) {
    point_array[i] = path[i].position;
  }
  return point_array;
}

std::vector<Point2D> LatticePlanner::convertObstacleToPoints(
    const derived_object_msgs::Object &msg) {
  ROS_INFO("Processing obstacles.");

  std::vector<Point2D> obstacle_pts(8, Point2D());

  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  double z = msg.pose.position.z;

  geometry_msgs::Quaternion geo_quat = msg.pose.orientation;

  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(geo_quat, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  double xrad = msg.shape.dimensions[0] / 2.0;
  double yrad = msg.shape.dimensions[1] / 2.0;
  double zrad = 0.8;

  Eigen::MatrixXd cpos(8, 2), rotyaw(2, 2), cpos_shift(8, 2);

  cpos << -xrad, -yrad, -xrad, 0, -xrad, yrad, 0, yrad, xrad, yrad, xrad, 0,
      xrad, -yrad, 0, -yrad;

  rotyaw << cos(yaw), sin(yaw), -sin(yaw), cos(yaw);

  cpos_shift << x, y, x, y, x, y, x, y, x, y, x, y, x, y, x, y;

  cpos = cpos * rotyaw + cpos_shift;

  for (int i = 0; i < cpos.rows(); i++) {
    obstacle_pts[i] = {cpos(i, 0), cpos(i, 1)};
  }

  return obstacle_pts;
}

std::vector<Pose2D>
LatticePlanner::convertFromNavPath(const nav_msgs::Path &msg) {
  std::vector<Pose2D> lane(0);
  for (int i = 0; i < msg.poses.size(); i++) {
    double x = msg.poses[i].pose.position.x;
    double y = msg.poses[i].pose.position.y;

    geometry_msgs::Quaternion geo_quat = msg.poses[i].pose.orientation;
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(geo_quat, quat);
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // skip overlapped or too close points
    if (lane.size() == 0) {
      lane.push_back(Pose2D(x, y, yaw));
    } else if ((lane.size() > 0) &&
               !(abs(x - lane[lane.size() - 1].position.x) < 0.01 &&
                 abs(y - lane[lane.size() - 1].position.y) < 0.01)) {
      lane.push_back(Pose2D(x, y, yaw));
    }
  }
  return lane;
}

/////////////////////////
// Callback Functions
/////////////////////////
void LatticePlanner::currentPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  // update current pose
  current_state_.position.x = msg->pose.position.x;
  current_state_.position.y = msg->pose.position.y;

  // get yaw from quaternion
  geometry_msgs::Quaternion geo_quat = msg->pose.orientation;
  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(geo_quat, quat);
  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  current_state_.yaw = yaw;

  // update path height
  path_height_ = msg->pose.position.z;

  ROS_INFO("Current state: x: %.2f, y: %.2f, yaw: %.2f",
           current_state_.position.x, current_state_.position.y,
           current_state_.yaw);
}

void LatticePlanner::currentSpeedCallback(
    const std_msgs::Float32::ConstPtr &msg) {
  current_state_.vel = msg->data;
  ROS_INFO("Current speed: %.2f", current_state_.vel);
}

void LatticePlanner::detectedObjectsCallback(
    const derived_object_msgs::ObjectArray::ConstPtr &msg) {
  ROS_INFO("Received dectected objects in lattice planner ...");
  detected_objects_ = msg->objects;
  obstacles_.resize(detected_objects_.size());
  for (int i = 0; i < detected_objects_.size(); i++) {
    std::vector<Point2D> obstacle_pts =
        convertObstacleToPoints(detected_objects_[i]);
    obstacles_[i] = obstacle_pts;
  }
}

void LatticePlanner::globalRouteCallback(const nav_msgs::Path::ConstPtr &msg) {
  // ROS_INFO("Global route got ...");
  global_route_ = convertFromNavPath(*msg);
  // ROS_INFO("Received %d valid points in the global route.",
  // int(global_route_.size()));
}

void LatticePlanner::stoplineSignalCallback(const std_msgs::Bool::ConstPtr &msg) {
  // ROS_INFO("Stop line signal got ...");
  if (!tl_detector_initialized_) {
    tl_detector_initialized_ = true;
  }
  if(msg->data && stop_point_initialized_) {
    if (stop_point_.x > 0 && distance2D(current_state_.position, stop_point_) < 8.0) {
      emergency_stop_ = true;
    }
  }
  else {
    emergency_stop_ = false;
  }
}

void LatticePlanner::stopPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
  // ROS_INFO("Stop point got ...");
  stop_point_initialized_ = true;
  stop_point_.x = msg->point.x;
  stop_point_.y = msg->point.y;
}

/////////////////////////
// Publish msgs
/////////////////////////
void LatticePlanner::publishSelectedPath(std::vector<Pose2D> &selected_path) {
  //////////////////////////////////////////////////////////////////////
  // Publish the final path as an output to other velocity planners
  //////////////////////////////////////////////////////////////////////
  nav_msgs::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = ros::Time::now();
  for (int i = 0; i < selected_path.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = selected_path[i].position.x;
    pose.pose.position.y = selected_path[i].position.y;
    pose.pose.position.z = path_height_ + 0.25; // use a hard coded value?
    path.poses.push_back(pose);
  }
  selected_path_pub_.publish(path);
}

void LatticePlanner::publishGenratedPaths(

    std::vector<std::vector<Pose2D>> &paths, int selected_index,
    std::vector<bool> &collision_check_array) {
  //////////////////////////////////////////////////////////////////////
  // Publish generated paths for visualization
  //////////////////////////////////////////////////////////////////////
  visualization_msgs::MarkerArray generate_paths_marker;
  for (int i = 0; i < paths.size(); i++) {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = local_frame_;
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "generated_paths";
    line_strip.id = i;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    // LINE_STRIP markers use only the x component of scale, for the line
    // width
    line_strip.scale.x = 0.25;

    if (i == selected_index) {
      // Line strip is pink
      line_strip.color.r = 1.0;
      line_strip.color.g = 0.078;
      line_strip.color.b = 0.576;
      line_strip.color.a = 1.0;
    } else {
      // Line strip is blue
      line_strip.color.b = 1.0;
      if (collision_check_array[i]) {
        line_strip.color.a = 1.0;
      } else {
        line_strip.color.a = 0.25;
      }
    }

    for (int j = 0; j < paths[i].size(); j++) {
      // convert to local for better visualization
      Point2D p_local = globalToLocal(current_state_.position,
                                      current_state_.yaw, paths[i][j].position);
      geometry_msgs::Point p;
      p.x = p_local.x;
      p.y = p_local.y;

      p.z = path_height_ + 0.5;
      line_strip.points.push_back(p);
    }

    generate_paths_marker.markers.push_back(line_strip);
  }
  generated_paths_marker_pub_.publish(generate_paths_marker);
}

void LatticePlanner::publishFinalWaypoints(
    std::vector<Waypoint2D> &vel_profile) {
  //////////////////////////////////////////////////////////////////////
  // Publish the final waypoints as an output to the controller
  //////////////////////////////////////////////////////////////////////
  casper_auto_msgs::WaypointArray final_waypoints;
  final_waypoints.header.frame_id = global_frame_;
  final_waypoints.header.stamp = ros::Time::now();
  for (int i = 0; i < vel_profile.size(); i++) {
    casper_auto_msgs::Waypoint wp;
    wp.pose.pose.position.x = vel_profile[i].position.x;
    wp.pose.pose.position.y = vel_profile[i].position.y;
    wp.twist.twist.linear.x = vel_profile[i].vel;
    final_waypoints.waypoints.push_back(wp);
  }
  final_waypoints_pub_.publish(final_waypoints);
}

void LatticePlanner::publishObstaclePolygons(

    std::vector<std::vector<Point2D>> &obstacles) {
  //////////////////////////////////////////////////////////////////////
  // Display obstacles as polygons (in local frame)
  //////////////////////////////////////////////////////////////////////
  jsk_recognition_msgs::PolygonArray polygon_array;
  polygon_array.header.frame_id = local_frame_;
  polygon_array.header.stamp = ros::Time::now();
  for (auto &obs : obstacles) {
    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = local_frame_;
    polygon.header.stamp = ros::Time::now();
    for (int i = 0; i < obs.size(); i++) {
      geometry_msgs::Point32 pt;
      pt.x = obs[i].x;
      pt.y = obs[i].y;
      pt.z = 2;
      polygon.polygon.points.push_back(pt);
    }
    polygon_array.polygons.push_back(polygon);
  }
  polygon_array_pub_.publish(polygon_array);
}

} // namespace planning
