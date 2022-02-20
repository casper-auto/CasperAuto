#include <math.h>

#include <cmath>
#include <iostream>
#include <string>

#include "carla_msgs/CarlaEgoVehicleControl.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "motion_controller.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_datatypes.h"
#include "casper_auto_msgs/VehicleState.h"
#include "casper_auto_msgs/Waypoint.h"
#include "casper_auto_msgs/WaypointArray.h"

using namespace std;

const double INTERP_DISTANCE_RES = 0.01;

vector<double> ego_state(4);
double ego_z = 0;
vector<vector<double>> final_waypoints;
vector<double> prev_p = {0, 0};

void egoOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // ROS_INFO("ego odometry got ...");

  geometry_msgs::Quaternion geo_quat = msg->pose.pose.orientation;

  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(geo_quat, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  ego_state[0] = msg->pose.pose.position.x;
  ego_state[1] = msg->pose.pose.position.y;
  ego_state[2] = yaw;
  double vx = msg->twist.twist.linear.x;
  double vy = msg->twist.twist.linear.y;
  ego_state[3] = std::hypot(vx, vy);

  ego_z = msg->pose.pose.position.z;

  // ROS_INFO("Ego state: x: %.2f, y: %.2f, yaw: %.2f, vel: %.2f", ego_state[0],
  // ego_state[1], ego_state[2], ego_state[3]);
}

void egoStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // ROS_INFO("ego state got ...");

  geometry_msgs::Quaternion geo_quat = msg->pose.pose.orientation;

  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(geo_quat, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  ego_state[0] = msg->pose.pose.position.x;
  ego_state[1] = msg->pose.pose.position.y;
  ego_state[2] = yaw;
  double vx = msg->twist.twist.linear.x;
  double vy = msg->twist.twist.linear.y;
  ego_state[3] = std::hypot(vx, vy);

  ego_z = msg->pose.pose.position.z;

  // ROS_INFO("Ego state: x: %.2f, y: %.2f, yaw: %.2f, vel: %.2f", ego_state[0],
  // ego_state[1], ego_state[2], ego_state[3]);
}

void finalWaypointsCallback(const casper_auto_msgs::WaypointArray::ConstPtr& msg) {
  // ROS_INFO("final waypoints plan got ...");
  ////////////////////////////////////////////////////////////////////////////
  // process final path
  ////////////////////////////////////////////////////////////////////////////
  final_waypoints.resize(msg->waypoints.size());
  for (int i = 0; i < msg->waypoints.size(); i++) {
    vector<double> p(3);
    p[0] = msg->waypoints[i].pose.pose.position.x;
    p[1] = msg->waypoints[i].pose.pose.position.y;
    p[2] = msg->waypoints[i].twist.twist.linear.x;
    final_waypoints[i] = p;
  }
  // ROS_INFO("First waypoint: x: %.2f, y: %.2f", final_waypoints[0][0],
  // final_waypoints[0][1]);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_planner");
  ros::NodeHandle n("~");

  //////////////////////////////////////////////////////////////////////////////
  // ROS Parameter server
  //////////////////////////////////////////////////////////////////////////////
  string role_name, control_method;
  double lookahead_t_mpc, lookahead_dist_mpc;
  n.param<string>("role_name", role_name, "ego_vehicle");
  n.param<string>("control_method", control_method, "MPC");
  n.param<double>("lookahead_t_mpc", lookahead_t_mpc, 2.0);
  n.param<double>("lookahead_dist_mpc", lookahead_dist_mpc, 5.0);

  bool add_latency;
  // Latency variables
  if (!n.getParam("add_latency", add_latency)) {
    add_latency = true;
  }

  double latency_steering, latency_brake, latency_throttle;
  if (!n.getParam("latency_throttle", latency_throttle)) {
    latency_throttle = 0.5;
  }

  if (!n.getParam("latency_brake", latency_brake)) {
    latency_brake = 0.2;
  }

  if (!n.getParam("latency_steering", latency_steering)) {
    latency_steering = 0.5;
  }

  double lag_throttle, lag_brake, lag_steering;
  if (!n.getParam("lag_throttle", lag_throttle)) {
    lag_throttle = 1.0;
  }

  if (!n.getParam("lag_brake", lag_brake)) {
    lag_brake = 1.0;
  }

  if (!n.getParam("lag_steering", lag_steering)) {
    lag_steering = 0.2;
  }

  double frequency_rate = 20.0;
  ros::Rate rate(frequency_rate);

  //////////////////////////////////////////////////////////////////////////////
  // ROS Subscriber and Publisher
  //////////////////////////////////////////////////////////////////////////////
  ros::Subscriber ego_state_sub =
      n.subscribe("/casper_auto/localization/odometry", 1000, egoStateCallback);
  ros::Subscriber final_waypoints_sub =
      n.subscribe("/casper_auto/planning/final_waypoints", 1000, finalWaypointsCallback);

  ros::Publisher vehicle_control_cmd_pub =
      n.advertise<carla_msgs::CarlaEgoVehicleControl>(
          "/carla/" + role_name + "/vehicle_control_cmd", 1000);

  MotionController mc(control_method, lookahead_dist_mpc, lookahead_t_mpc);

  std::deque<double> latency_steering_deque, latency_brake_deque,
      latency_throttle_deque;
  double max_latency =
      std::max(std::max(latency_brake, latency_steering), latency_throttle);
  int max_size_queue =
      std::max(int(std::ceil(max_latency * frequency_rate * 1.5)), 15);

  while (ros::ok()) {
    double current_timestamp = ros::Time::now().toSec();
    vector<double> cmd(3);
    vector<double> wp_distance;
    vector<vector<double>> wp_interp;

    // if ego car is on the air, re-initialize pid
    double dist = hypot(ego_state[0] - prev_p[0], ego_state[1] - prev_p[1]);

    if (dist > 10.0) {
      mc.reset_all_vars();
      final_waypoints.clear();
      std::cout << "Restarting ego car" << std::endl;
    }

    prev_p[0] = ego_state[0];
    prev_p[1] = ego_state[1];

    if (final_waypoints.size() > 1) {
      ////////////////////////////////////////////////////////////////////////////
      // Downsize the waypoints when they are too dense
      ////////////////////////////////////////////////////////////////////////////
      vector<vector<double>> new_waypoints;
      new_waypoints.push_back(final_waypoints[0]);
      for (int i = 1; i < final_waypoints.size(); i++) {
        double dist = norm(
            {final_waypoints[i][0] - new_waypoints[new_waypoints.size() - 1][0],
             final_waypoints[i][1] -
                 new_waypoints[new_waypoints.size() - 1][1]});
        if (dist >= INTERP_DISTANCE_RES)
          new_waypoints.push_back(final_waypoints[i]);
      }
      final_waypoints = new_waypoints;
    }

    if (final_waypoints.size() > 1) {
      ////////////////////////////////////////////////////////////////////////////
      // Linear interpolation computation on the waypoints
      ////////////////////////////////////////////////////////////////////////////
      for (int i = 1; i < final_waypoints.size(); i++) {
        wp_distance.push_back(
            norm({final_waypoints[i][0] - final_waypoints[i - 1][0],
                  final_waypoints[i][1] - final_waypoints[i - 1][1]}));
      }
      wp_distance.push_back(0);

      for (int i = 0; i < final_waypoints.size() - 1; i++) {
        wp_interp.push_back(final_waypoints[i]);
        int num_pts_to_interp = floor(wp_distance[i] / INTERP_DISTANCE_RES) - 1;
        vector<double> wp_vector = {
            final_waypoints[i + 1][0] - final_waypoints[i][0],
            final_waypoints[i + 1][1] - final_waypoints[i][1],
            final_waypoints[i + 1][2] - final_waypoints[i][2]};
        double vector_len =
            sqrt(wp_vector[0] * wp_vector[0] + wp_vector[1] * wp_vector[1]);
        vector<double> wp_uvector = {wp_vector[0] / vector_len,
                                     wp_vector[1] / vector_len,
                                     wp_vector[2] / vector_len};
        for (int j = 0; j < num_pts_to_interp; j++) {
          vector<double> next_wp_vector = {
              INTERP_DISTANCE_RES * float(j + 1) * wp_uvector[0],
              INTERP_DISTANCE_RES * float(j + 1) * wp_uvector[1],
              INTERP_DISTANCE_RES * float(j + 1) * wp_uvector[2]};
          vector<double> next_addin = {
              final_waypoints[i][0] + next_wp_vector[0],
              final_waypoints[i][1] + next_wp_vector[1],
              final_waypoints[i][2] + next_wp_vector[2]};
          wp_interp.push_back(next_addin);
        }
      }
      wp_interp.push_back(final_waypoints[final_waypoints.size() - 1]);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Shift Speed Profile
    ////////////////////////////////////////////////////////////////////////////
    int shift_step = min(20, int(wp_interp.size()));
    for (int i = 0; i < wp_interp.size() - shift_step; i++) {
      wp_interp[i][2] = wp_interp[i + shift_step][2];
    }

    ////////////////////////////////////////////////////////////////////////////
    // Controller Update
    ////////////////////////////////////////////////////////////////////////////
    if (wp_interp.size() > 1) {
      mc.update_waypoints(wp_interp);
      mc.update_values(ego_state, current_timestamp);
      mc.update_controls(lag_throttle, lag_brake, lag_steering, frequency_rate,
                         add_latency);
      cmd = mc.get_commands();
    } else {
      cmd = {0.0, 0.0, 1};
    }

    // ROS_INFO("get commands:");
    // ROS_INFO("throttle: %2f, steer: %2f, brake: %2f", cmd[0], cmd[1],
    // cmd[2]);

    ////////////////////////////////////////////////////////////////////////////
    // Publish commands
    ////////////////////////////////////////////////////////////////////////////
    carla_msgs::CarlaEgoVehicleControl control_cmd;
    control_cmd.throttle = cmd[0];
    control_cmd.steer = cmd[1];
    control_cmd.brake = cmd[2];
    control_cmd.hand_brake = false;
    control_cmd.reverse = false;

    if (add_latency) {
      // Add new command to the queue considering max size
      // STEER
      if (latency_steering_deque.size() == max_size_queue) {
        latency_steering_deque.pop_front();
      }
      latency_steering_deque.push_back(control_cmd.steer);

      int index_pick_steering =
          int(std::ceil(latency_steering * frequency_rate));

      if (index_pick_steering > latency_steering_deque.size()) {
        index_pick_steering = latency_steering_deque.size();
      }

      // THROTTLE
      if (latency_throttle_deque.size() == max_size_queue) {
        latency_throttle_deque.pop_front();
      }
      latency_throttle_deque.push_back(control_cmd.throttle);

      int index_pick_throttle =
          int(std::ceil(latency_throttle * frequency_rate));

      if (index_pick_throttle > latency_throttle_deque.size()) {
        index_pick_throttle = latency_throttle_deque.size();
      }

      // BRAKE
      if (latency_brake_deque.size() == max_size_queue) {
        latency_brake_deque.pop_front();
      }
      latency_brake_deque.push_back(control_cmd.brake);

      int index_pick_brake = int(std::ceil(latency_brake * frequency_rate));

      if (index_pick_brake > latency_brake_deque.size()) {
        index_pick_brake = latency_brake_deque.size();
      }

      carla_msgs::CarlaEgoVehicleControl control_cmd_send = control_cmd;

      control_cmd_send.steer =
          latency_steering_deque[latency_steering_deque.size() -
                                 index_pick_steering];
      control_cmd_send.throttle =
          latency_throttle_deque[latency_throttle_deque.size() -
                                 index_pick_throttle];
      control_cmd_send.brake =
          latency_brake_deque[latency_brake_deque.size() - index_pick_brake];

      vehicle_control_cmd_pub.publish(control_cmd_send);
    } else {
      vehicle_control_cmd_pub.publish(control_cmd);
    }

    ros::spinOnce();
    rate.sleep();
    // ROS_INFO("The iteration end.");
  }

  return 0;
}
