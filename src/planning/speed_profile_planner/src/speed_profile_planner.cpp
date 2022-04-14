#include <cmath>
#include <iostream>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <casper_auto_msgs/PerceptionLanes.h>
#include <casper_auto_msgs/PredictionArray.h>
#include <casper_auto_msgs/VehicleState.h>
#include <casper_auto_msgs/Waypoint.h>
#include <casper_auto_msgs/WaypointArray.h>

#include "speed_planner_lib/speed_planner.h"

using namespace std;
using namespace planning;

const double INTERP_DISTANCE_RES = 0.1;

vector<Point2D> final_path, global_path, target_lane_line;
bool ghost_ahead = false;
vector<vector<double>> ghost_pts;
vector<Obstacle> vehicles, pedestrians;

bool use_ghost = false;

vector<vector<double>> stop_sign;
bool is_stop_sign_set = false;

vector<vector<double>> cross_walk;
bool is_cross_walk_set = false;

vector<double> pre_state(3);
vector<double> ego_state(3);

vector<double> current_pose(3);

double current_speed;

double ego_theta;

double norm(vector<double> a) {
  double sum = 0;
  for (int i = 0; i < a.size(); i++) {
    sum += a[i] * a[i];
  }
  return sqrt(sum);
}

double constrainAngle(double x) {
  x = fmod(x, M_PI * 2);
  if (x < 0)
    x += M_PI * 2;
  return x;
}

void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  ROS_INFO("Received the current_pose ...");

  geometry_msgs::Quaternion geo_quat = msg->pose.orientation;

  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(geo_quat, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  current_pose[0] = msg->pose.position.x;
  current_pose[1] = msg->pose.position.y;
  current_pose[2] = yaw;

  // ROS_INFO("Current pose: x: %.2f, y: %.2f, yaw: %.2f", current_pose[0],
  // current_pose[1], current_pose[2]);
}

void currentSpeedCallback(const std_msgs::Float32::ConstPtr &msg) {
  ROS_INFO("Received the current_speed ...");
  current_speed = msg->data;
  // ROS_INFO("Current speed: %.2f", current_speed);
}

void finalPathCallback(const nav_msgs::Path::ConstPtr &msg) {
  ROS_INFO("Ego vehicle's final path got.");

  final_path.resize(0);

  for (int i = 0; i < msg->poses.size(); i++) {
    double x = msg->poses[i].pose.position.x;
    double y = msg->poses[i].pose.position.y;

    final_path.push_back(Point2D(x, y));
  }

  ROS_INFO("Received %d valid points in the final path.",
           int(final_path.size()));
}

void globalRouteCallback(const nav_msgs::Path::ConstPtr &msg) {
  ROS_INFO("Global path got.");

  global_path.resize(0);

  for (int i = 0; i < msg->poses.size(); i++) {
    double x = msg->poses[i].pose.position.x;
    double y = msg->poses[i].pose.position.y;

    global_path.push_back(Point2D(x, y));
  }

  ROS_INFO("Received %d valid points in the final path.",
           int(global_path.size()));
}

vector<vector<double>>
extractObstaclePoints(const casper_auto_msgs::VehicleState &obstacle) {
  vector<vector<double>> obstacle_pts(8, vector<double>(2));

  // ROS_INFO("Processing obstacles ...");

  double x = obstacle.pose.pose.position.x;
  double y = obstacle.pose.pose.position.y;
  double z = obstacle.pose.pose.position.z;

  geometry_msgs::Quaternion geo_quat = obstacle.pose.pose.orientation;

  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(geo_quat, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  double xrad = obstacle.length / 2.0;
  double yrad = obstacle.width / 2.0;
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

void vehiclesPredictionCallback(const casper_auto_msgs::PredictionArray::ConstPtr &msg) {
  ROS_INFO("Vehicles with prediction got.");

  vehicles.resize(0);

  ObstacleType obs_type = CAR;

  for (casper_auto_msgs::Prediction pred : msg->predictions) {
    if (!use_ghost && pred.agent_id == 123456789)
      continue;

    casper_auto_msgs::WaypointArray pred_traj =
        pred.trajectories[0].trajectory_estimated;
    double dt = pred.dt;

    std::vector<TrajectoryPoint> obs_traj;
    for (int i = 0; i < pred_traj.waypoints.size(); i++) {
      TrajectoryPoint traj_point;
      Point2D point_in;
      point_in.x = pred_traj.waypoints[i].pose.pose.position.x;
      point_in.y = pred_traj.waypoints[i].pose.pose.position.y;
      Point2D point_out = localToGlobal(Point2D(current_pose[0], current_pose[1]), current_pose[2], point_in);
      traj_point.x = point_out.x;
      traj_point.y = point_out.y;
      double vx = pred_traj.waypoints[i].twist.twist.linear.x;
      double vy = pred_traj.waypoints[i].twist.twist.linear.y;
      traj_point.v = sqrt(vx * vx + vy * vy);
      traj_point.v_limit = 5.0;
      obs_traj.push_back(traj_point);
    }

    // check if a target lane vehicle, otherwise it is in the other lane
    bool in_target_lane = false;
    if (target_lane_line.size() > 0) {
      Point2D check_p(obs_traj[0].x, obs_traj[0].y);
      in_target_lane = isOnPath(target_lane_line, check_p, 2.0);
    }

    Obstacle obs(obs_type, obs_traj, in_target_lane);

    vehicles.push_back(obs);
  }
}

void pedestriansPredictionCallback(
    const casper_auto_msgs::PredictionArray::ConstPtr &msg) {
  ROS_INFO("Pedestrians with prediction got.");

  pedestrians.resize(0);

  ObstacleType obs_type = PEDESTRIAN;

  for (casper_auto_msgs::Prediction pred : msg->predictions) {
    casper_auto_msgs::WaypointArray pred_traj =
        pred.trajectories[0].trajectory_estimated;
    double dt = pred.dt;

    std::vector<TrajectoryPoint> obs_traj;
    for (int i = 0; i < pred_traj.waypoints.size(); i++) {
      TrajectoryPoint traj_point;
      Point2D point_in;
      point_in.x = pred_traj.waypoints[i].pose.pose.position.x;
      point_in.y = pred_traj.waypoints[i].pose.pose.position.y;
      Point2D point_out = localToGlobal(Point2D(current_pose[0], current_pose[1]), current_pose[2], point_in);
      traj_point.x = point_out.x;
      traj_point.y = point_out.y;
      double vx = pred_traj.waypoints[i].twist.twist.linear.x;
      double vy = pred_traj.waypoints[i].twist.twist.linear.y;
      traj_point.v = sqrt(vx * vx + vy * vy);
      traj_point.v_limit = 3.5;
      obs_traj.push_back(traj_point);
    }
    Obstacle obs(obs_type, obs_traj);

    pedestrians.push_back(obs);
  }
}

void stopsignCallback(const visualization_msgs::MarkerArray::ConstPtr &msg) {
  ROS_INFO("Stopsign markers got.");

  stop_sign.resize(0);

  if (msg->markers.size() > 0) {
    for (geometry_msgs::Point p : msg->markers[0].points) {
      double x = p.x;
      double y = p.y;
      vector<double> point = {x, y};
      stop_sign.push_back(point);
    }
  }
}

void crosswalkCallback(const visualization_msgs::MarkerArray::ConstPtr &msg) {
  ROS_INFO("Crosswalk markers got.");

  cross_walk.resize(0);

  if (msg->markers.size() > 0) {
    for (geometry_msgs::Point p : msg->markers[0].points) {
      double x = p.x;
      double y = p.y;
      vector<double> point = {x, y};
      cross_walk.push_back(point);
    }
  }
}

void targetLaneCallback(const visualization_msgs::Marker::ConstPtr &msg) {
  ROS_INFO("Target lane line marker got.");

  target_lane_line.resize(0);

  for (geometry_msgs::Point p : msg->points) {
    double x = p.x;
    double y = p.y;

    Point2D p_local(x, y);
    Point2D p_global =
        localToGlobal(Point2D(ego_state[0], ego_state[1]), ego_theta, p_local);

    target_lane_line.push_back(p_global);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "speed_profile_planner");
  ros::NodeHandle n("~");

  // Get param
  bool left_hand;
  bool low_speed_cut;
  double low_speed_thresh;
  double speed_limit;
  int viz_mode;
  int velocity_offset;
  double car_following_distance;
  double oncoming_car_distance;
  double stop_line_offset;
  double stop_release_time;
  double crosswalk_span;
  double crosswalk_shift;

  double s_horizon;
  double t_horizon;
  double dt;

  MPCConfig *mpc_config = new MPCConfig();

  // Params
  n.param<bool>("use_ghost", use_ghost, false);
  n.param<bool>("left_hand", left_hand, false);
  n.param<bool>("low_speed_cut", low_speed_cut, false);
  n.param<double>("low_speed_thresh", low_speed_thresh, 0.5);
  n.param<double>("speed_limit", speed_limit, 5.0);
  n.param<int>("velocity_offset", velocity_offset, 2);
  n.param<double>("car_following_distance", car_following_distance, 10);
  n.param<double>("oncoming_car_distance", oncoming_car_distance, 15);
  n.param<double>("stop_line_offset", stop_line_offset, 0.0);
  n.param<double>("stop_release_time", stop_release_time, 3);
  n.param<double>("crosswalk_span", crosswalk_span, 2.0);
  n.param<double>("crosswalk_shift", crosswalk_shift, 1.0);
  n.param<int>("viz_mode", viz_mode, 0);

  // ST Graph Params
  n.param<double>("st_graph/s_horizon", s_horizon, 20.0);
  n.param<double>("st_graph/t_horizon", t_horizon, 2.0);
  n.param<double>("st_graph/t_step", dt, 0.1);

  // MPC Config Params
  n.param<int>("mpc_config/n_steps", mpc_config->n_steps, 50);
  n.param<double>("mpc_config/dt", mpc_config->dt, 1.0);
  n.param<double>("mpc_config/planning_dt", mpc_config->planning_dt, 0.1);
  n.param<string>("mpc_config/max_iteration", mpc_config->max_iteration, "200");
  n.param<string>("mpc_config/tolerance", mpc_config->tolerance, "1e-6");
  n.param<double>("mpc_config/displacement_weight",
                  mpc_config->displacement_weight, 10.0);
  n.param<double>("mpc_config/vel_weight", mpc_config->vel_weight, 100.0);
  n.param<double>("mpc_config/acc_weight", mpc_config->acc_weight, 1.0);
  n.param<double>("mpc_config/acc_dot_weight", mpc_config->acc_dot_weight,
                  100.0);
  n.param<double>("mpc_config/throttle_brake_weight",
                  mpc_config->throttle_brake_weight, 100.0);
  n.param<double>("mpc_config/default_limit", mpc_config->default_limit,
                  1.0e19);
  n.param<bool>("mpc_config/short_log_m", mpc_config->mpc_short_log_m, false);
  n.param<bool>("mpc_config/long_log_m", mpc_config->mpc_long_log_m, false);
  n.param<bool>("mpc_config/cost_log_m", mpc_config->mpc_cost_log_m, false);
  mpc_config->s_dot_max = speed_limit;

  // Subscribers
  ros::Subscriber current_pose_sub =
      n.subscribe("/current_pose", 10, currentPoseCallback);
  ros::Subscriber current_speed_sub =
      n.subscribe("/current_speed", 10, currentSpeedCallback);
  ros::Subscriber final_path_sub =
      n.subscribe("/final_path", 10, finalPathCallback);
  ros::Subscriber global_route_sub =
      n.subscribe("/global_route", 10, globalRouteCallback);

  ros::Subscriber vehicles_with_prediction_sub =
      n.subscribe("/prediction/vehicles", 10, vehiclesPredictionCallback);
  ros::Subscriber pedestrians_with_prediction_sub =
      n.subscribe("/prediction/pedestrians", 10, pedestriansPredictionCallback);

  ros::Subscriber stopsign_marker_sub =
      n.subscribe("/stopsign_marker", 10, stopsignCallback);
  ros::Subscriber crosswalk_marker_sub =
      n.subscribe("/crosswalk_marker", 10, crosswalkCallback);
  ros::Subscriber target_lane_marker_sub =
      n.subscribe("/further_lane_line_marker", 10, targetLaneCallback);

  // Publishers
  ros::Publisher final_waypoints_pub =
      n.advertise<casper_auto_msgs::WaypointArray>(
          "/final_waypoints", 10);
  ros::Publisher ghost_ahead_polygon_pub =
      n.advertise<geometry_msgs::PolygonStamped>("/ghost_ahead_polygon",
                                                 10);

  ros::Rate rate(10);

  // speed planner
  SpeedPlanner sp(mpc_config, left_hand, low_speed_cut, low_speed_thresh,
                  velocity_offset, car_following_distance,
                  oncoming_car_distance, stop_line_offset, stop_release_time,
                  crosswalk_span, crosswalk_shift, viz_mode);

  // --------------- Main Loop ---------------
  while (ros::ok()) {
    ////////////////////////////////////////////////////////////////////////////
    // Run every single second
    ////////////////////////////////////////////////////////////////////////////
    std::vector<TrajectoryPoint> reference_trajectory;

    if (!final_path.empty()) {
      // --------------- SpeedPlanner ---------------
      vector<Obstacle> obstacles;

      for (auto veh : vehicles) {
        obstacles.push_back(veh);
      }

      for (auto ped : pedestrians) {
        obstacles.push_back(ped);
      }

      // STGraph st_graph(final_path, obstacles, s_horizon, t_horizon, dt);
      // std::vector<STArea> areas = st_graph.GetAllOccupied();

      // construct st_cell_planner
      // STCellPlanner st_cell_planner(st_graph, dt);
      // std::vector<std::vector<STCell>> candidate_plans =
      // st_cell_planner.SearchCandidatePlans();
      //
      // cout << "The number of solutions: " << candidate_plans.size() << endl;
      // PrintResult(candidate_plans);
      // SaveResult(candidate_plans);

      reference_trajectory.resize(0);

      vector<double> wp_distance;
      vector<Point2D> fp_interp;

      if (final_path.size() > 1) {
        ////////////////////////////////////////////////////////////////////////////
        // Linear interpolation computation on the waypoints
        ////////////////////////////////////////////////////////////////////////////
        for (int i = 1; i < final_path.size(); i++) {
          wp_distance.push_back(norm({final_path[i].x - final_path[i - 1].x,
                                      final_path[i].y - final_path[i - 1].y}));
        }
        wp_distance.push_back(0);

        for (int i = 0; i < final_path.size() - 1; i++) {
          fp_interp.push_back({final_path[i].x, final_path[i].y});
          int num_pts_to_interp =
              floor(wp_distance[i] / INTERP_DISTANCE_RES) - 1;
          vector<double> wp_vector = {final_path[i + 1].x - final_path[i].x,
                                      final_path[i + 1].y - final_path[i].y};
          double vector_len =
              sqrt(wp_vector[0] * wp_vector[0] + wp_vector[1] * wp_vector[1]);
          vector<double> wp_uvector = {wp_vector[0] / vector_len,
                                       wp_vector[1] / vector_len};
          for (int j = 0; j < num_pts_to_interp; j++) {
            vector<double> next_wp_vector = {
                INTERP_DISTANCE_RES * float(j + 1) * wp_uvector[0],
                INTERP_DISTANCE_RES * float(j + 1) * wp_uvector[1]};
            vector<double> next_addin = {final_path[i].x + next_wp_vector[0],
                                         final_path[i].y + next_wp_vector[1]};
            fp_interp.push_back({next_addin[0], next_addin[1]});
          }
        }
        fp_interp.push_back(final_path[final_path.size() - 1]);
      }

      for (auto e : final_path) {
        TrajectoryPoint p;
        p.x = e.x;
        p.y = e.y;
        p.v = 0;
        p.v_limit = speed_limit;
        reference_trajectory.push_back(p);
      }

      // Set current velocity
      reference_trajectory[0].v = ego_state[2];

      sp.Plan(obstacles, reference_trajectory);

      // Set stop sign
      if (!is_stop_sign_set && stop_sign.size() > 0) {
        sp.SetStopSign(stop_sign);
        is_stop_sign_set = true;
      }

      // Set cross walk
      if (!is_cross_walk_set && cross_walk.size() > 0) {
        sp.SetCrossWalk(cross_walk);
        is_cross_walk_set = true;
      }

      // Check if restart
      double motion_dist =
          distance2D(pre_state[0], pre_state[1], ego_state[0], ego_state[1]);
      if (motion_dist > 10) {
        sp.ResetStopSign();
      }

      pre_state = ego_state;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Publish the ROS message containing the waypoints to the controller
    ////////////////////////////////////////////////////////////////////////////
    casper_auto_msgs::WaypointArray waypoints;
    waypoints.header.frame_id = "map";
    waypoints.header.stamp = ros::Time::now();
    for (int i = 0; i < reference_trajectory.size(); i++) {
      casper_auto_msgs::Waypoint wp;
      wp.pose.pose.position.x = reference_trajectory[i].x;
      wp.pose.pose.position.y = reference_trajectory[i].y;
      wp.twist.twist.linear.x = reference_trajectory[i].v;
      waypoints.waypoints.push_back(wp);
    }

    final_waypoints_pub.publish(waypoints);

    ////////////////////////////////////////////////////////////////////////////
    // Display ghost as polygon
    ////////////////////////////////////////////////////////////////////////////
    if (ghost_ahead) {
      geometry_msgs::PolygonStamped polygon;
      polygon.header.frame_id = "map";
      polygon.header.stamp = ros::Time::now();
      for (int i = 0; i < ghost_pts.size(); i++) {
        geometry_msgs::Point32 pt;
        pt.x = ghost_pts[i][0];
        pt.y = ghost_pts[i][1];
        pt.z = 2;
        polygon.polygon.points.push_back(pt);
      }
      ghost_ahead_polygon_pub.publish(polygon);
    } else {
      geometry_msgs::PolygonStamped polygon;
      polygon.header.frame_id = "map";
      polygon.header.stamp = ros::Time::now();
      ghost_ahead_polygon_pub.publish(polygon);
    }

    ////////////////////////////////////////////////////////////////////////////
    // ROS Spin
    ////////////////////////////////////////////////////////////////////////////
    ros::spinOnce();
    rate.sleep();
    // ROS_INFO("The iteration end.");
  }

  return 0;
}
