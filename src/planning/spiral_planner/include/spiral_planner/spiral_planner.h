#ifndef SPIRAL_PLANNER_H
#define SPIRAL_PLANNER_H

#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <future>
#include <thread>

#include <Eigen/Dense> // path optimizer

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <derived_object_msgs/Object.h>
#include <derived_object_msgs/ObjectArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>

#include <casper_auto_msgs/CenterLanes.h>
#include <casper_auto_msgs/VehicleState.h>
#include <casper_auto_msgs/Waypoint.h>
#include <casper_auto_msgs/WaypointArray.h>

#include "geometry.hpp"
#include "state.h"
#include "frenet_frame.h"

#include "collision_checker/collision_checker.h"
#include "path_optimizer/path_optimizer.h"
#include "velocity_planner/velocity_planner.h"

namespace planning {

// global variables
#define G_LOOKAHEAD_MIN 8.0  // m
#define G_LOOKAHEAD_MAX 20.0 // m

enum DrivingMode { LaneKeep, LaneChange };

///////////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////////

double normalized_angle(double angle);

///////////////////////////////////////////////////////////////////////////////
// class SpiralPlanner
///////////////////////////////////////////////////////////////////////////////

class SpiralPlanner {
public:
  SpiralPlanner(ros::NodeHandle nh);

  ~SpiralPlanner();

private:
  // ros
  ros::NodeHandle nh_;

  // ros subscribers
  ros::Subscriber current_pose_sub_;
  ros::Subscriber current_speed_sub_;
  ros::Subscriber global_route_sub_;
  ros::Subscriber detected_objects_sub_;
  ros::Subscriber stop_line_signal_sub_;
  ros::Subscriber stop_point_sub_;

  // ros publishers
  ros::Publisher selected_path_pub_;
  ros::Publisher generated_paths_marker_pub_;
  ros::Publisher polygon_array_pub_;   // deprecated?
  ros::Publisher final_waypoints_pub_; // for control

  // vars loaded from ros params
  std::string global_frame_;
  std::string local_frame_;
  double lookahead_distance_;
  double path_distance_;
  int num_paths_;
  double lateral_offset_;
  double longitudinal_offset_;
  bool run_parallel_;
  // collision checker params
  bool enable_collision_checker_;
  std::vector<double> circle_offsets_;
  std::vector<double> circle_radii_;
  double path_select_weight_;
  std::vector<std::vector<Point2D>> obstacles_;
  // velocity planner params
  bool enable_velocity_planner_;
  double cruise_speed_;
  double time_step_;
  double accel_max_;
  double slow_speed_;
  // obstacles display flag
  bool enable_obstacles_display_;
  bool tl_detector_initialized_;

  // from callback functions
  State current_state_; // x, y, yaw, speed
  std::vector<Pose2D> global_route_;
  std::vector<Pose2D> reference_route_;
  std::vector<Pose2D> prev_selected_path_;
  std::vector<autoware_msgs::DetectedObject> detected_objects_;
  std::unordered_map<int, double> lane_widths_;
  Point2D stop_point_;
  bool stop_point_initialized_;

  // specify leader vehicle based on the picked path
  State leader_state_;
  bool follow_lead_vehicle_;
  bool emergency_stop_;

  // other vars
  int closest_index_;
  double closest_distance_;
  int lookahead_index_;
  Pose2D goal_pose_;
  double path_height_;
  double prev_timestamp_;
  DrivingMode driving_mode_;
  double goal_offset_;
  double shift_angle_;

  // instances
  PathOptimizer path_optimizer_;
  CollisionChecker collision_checker_;
  VelocityPlanner velocity_planner_;

private:
  void initialize();

  void plan();

  std::vector<Pose2D> getGoalPoseSet();

  std::vector<std::vector<Pose2D>>
  generatePaths(std::vector<Pose2D> &goal_pose_set);

  std::vector<Pose2D>
  convertToPoseArray(std::vector<std::vector<double>> &spiral);

  std::vector<std::vector<Pose2D>>
  transformPaths(std::vector<std::vector<Pose2D>> &paths);

  std::vector<Pose2D> extendPath(std::vector<Pose2D> &path,
                                 double path_distance, int path_id);

  int getClosestIndex(double x, double y, std::vector<Pose2D> &path);

  int getLookaheadIndex(double x, double y, std::vector<Pose2D> &path,
                        double lookahead_distance);

  std::vector<Point2D> convertToPointArray(std::vector<Pose2D> &path);

  std::vector<Point2D>
  convertObstacleToPoints(const autoware_msgs::DetectedObject &msg);

  std::vector<Pose2D> convertFromNavPath(const nav_msgs::Path &msg);

  // callback functions
  void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  void currentSpeedCallback(const std_msgs::Float32::ConstPtr &msg);

  void detectedObjectsCallback(
      const autoware_msgs::DetectedObjectArray::ConstPtr &msg);

  void globalRouteCallback(const nav_msgs::Path::ConstPtr &msg);

  void stoplineSignalCallback(const std_msgs::Bool::ConstPtr &msg);

  void stopPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg);

  // publish functions
  void publishSelectedPath(std::vector<Pose2D> &selected_path);

  void publishGenratedPaths(std::vector<std::vector<Pose2D>> &paths,
                            int selected_index,
                            std::vector<bool> &collision_check_array);

  void publishFinalWaypoints(std::vector<Waypoint2D> &vel_profile);

  void publishObstaclePolygons(std::vector<std::vector<Point2D>> &obstacles);
};

} // namespace planning

#endif /* SPIRAL_PLANNER_H */
