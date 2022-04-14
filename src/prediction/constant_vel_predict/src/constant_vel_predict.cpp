#include <constant_vel_predict/constant_vel_predict.h>
#include <tf/transform_datatypes.h>
#include <unordered_set>


////////////////////////////////////////////////////////////////////////////////
// Vehicles estimation class
////////////////////////////////////////////////////////////////////////////////

AgentsEstimation::AgentsEstimation() {
  // ROS Params
  nh_m.param<float>("plan_t", plan_t_m, 10.0f);
  nh_m.param<float>("dt", dt_m, 0.1f);
  np_m = uint16_t(std::round(plan_t_m / dt_m)) + 1;

  nh_m.param("/prediction/v_max_ag", max_speed_agts_m, 35.0);

  // subscribers
  ego_pose_sub_m = nh_m.subscribe(
      "/casper_auto/current_pose", 1, &AgentsEstimation::EgoPoseCallback, this);
  detected_objects_sub_m = nh_m.subscribe(
      "/casper_auto/detected_objects", 1, &AgentsEstimation::DetectedObjectsCallback, this);
  reference_lanes_sub_m = nh_m.subscribe(
      "/casper_auto/odr_map/reference_lanes", 1, &AgentsEstimation::ReferenceLanesCallback, this);

  // publishers
  vehicles_prediction_pub_m = nh_m.advertise<casper_auto_msgs::PredictionArray>(
      "/casper_auto/prediction/vehicles", 1);
  pedestrians_prediction_pub_m = nh_m.advertise<casper_auto_msgs::PredictionArray>(
      "/casper_auto/prediction/pedestrians", 1);

  // timer
  timer_m = nh_m.createTimer(ros::Duration(0.1), &AgentsEstimation::TimerCallback, this);

  // var initialization
  prev_t_objects_m = ros::Time::now();
  prev_t_err_objects_m = ros::Time::now();
  prev_t_err_lanes_m = ros::Time::now();
}

void AgentsEstimation::EgoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // ROS_INFO("Received the ego_pose ...");

  geometry_msgs::Quaternion geo_quat = msg->pose.orientation;

  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(geo_quat, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  ego_xy_m.x = msg->pose.position.x;
  ego_xy_m.y = msg->pose.position.y;
  ego_theta_m = yaw;
}

void AgentsEstimation::DetectedObjectsCallback(
    const derived_object_msgs::ObjectArray::ConstPtr &msg) {
  // ROS_INFO("Received a message of detected_objects in constant_vel_predict.");
  vehicle_vec_m.clear();
  pedestrian_vec_m.clear();
  for (auto& obj : msg->objects) {
    // vehicles
    if (obj.classification == derived_object_msgs::Object::CLASSIFICATION_CAR) {
        vehicle_vec_m.push_back(obj);
    }
    // pedestrian
    if (obj.classification == derived_object_msgs::Object::CLASSIFICATION_PEDESTRIAN ||
      obj.classification == derived_object_msgs::Object::CLASSIFICATION_BIKE) {
        pedestrian_vec_m.push_back(obj);
    }
  }
  prev_t_objects_m = ros::Time::now();
}

void AgentsEstimation::ReferenceLanesCallback(
    const casper_auto_msgs::CenterLanes::ConstPtr &msg) {
  // ROS_INFO("Received a message of reference_lanes in constant_vel_predict.");
  map_lanes_frenet_m.clear();

  for (uint16_t l = 0; l < msg->ids.size(); l++) {
    std::vector<geometry::Point2D> path(msg->center_lines[l].poses.size());
    for (uint32_t i = 0; i < msg->center_lines[l].poses.size(); i++) {
      path[i] =
          geometry::Point2D(msg->center_lines[l].poses[i].pose.position.x,
                            msg->center_lines[l].poses[i].pose.position.y);
    }
    map_lanes_frenet_m[msg->ids[l]] = geometry::Frenet(path);
  }
}

void AgentsEstimation::TimerCallback(const ros::TimerEvent &) {
  // Timeout times
  double timeout_not_rec = 5.0;
  double timeout_warn = 10.0;

  // Check for failure in perception
  bool valid_objects_msg = true, valid_lanes_msg = true;
  ros::Time t_now = ros::Time::now();

  if ((t_now - prev_t_objects_m).toSec() > timeout_not_rec) {
    valid_objects_msg = false;
    if ((t_now - prev_t_err_objects_m).toSec() > timeout_warn) {
      ROS_WARN("Did not get a detected_objects message");
      prev_t_err_objects_m = t_now;
    }
  }

  // Checks to validate input data. Lane data is needed for calculations
  if (map_lanes_frenet_m.empty()) {
    valid_lanes_msg = false;
    if ((t_now - prev_t_err_lanes_m).toSec() > timeout_warn) {
      ROS_WARN("Did not get a reference_lanes message");
      prev_t_err_lanes_m = t_now;
    }
  }

  if (valid_objects_msg && valid_lanes_msg) {
    casper_auto_msgs::PredictionArray vehicles_prediction_msg;
    vehicles_prediction_msg.header.stamp = ros::Time::now();
    PredictTrajectoryVehicles(vehicles_prediction_msg);
    vehicles_prediction_pub_m.publish(vehicles_prediction_msg);
  }

  if (valid_objects_msg) {
    casper_auto_msgs::PredictionArray pedestrians_prediction_msg;
    pedestrians_prediction_msg.header.stamp = ros::Time::now();
    PredictTrajectoryPedestrians(pedestrians_prediction_msg);
    pedestrians_prediction_pub_m.publish(pedestrians_prediction_msg);
  }
}

void AgentsEstimation::PredictTrajectoryVehicles(
    casper_auto_msgs::PredictionArray &vehicles_prediction_msg) {
  // Constant velocity for each car in that lane
  for (auto& veh : vehicle_vec_m) {
    geometry::Point2D p_in(veh.pose.position.x,
                           veh.pose.position.y);
    geometry::Point2D p_xy = localToGlobal(ego_xy_m, ego_theta_m, p_in);

    // find matched lane using frenet frame
    double min_d = 1000;
    double matched_lane = -1;
    for (auto& lane : map_lanes_frenet_m) {
      int lane_id = lane.first;
      geometry::Point_Frenet p_sd;
      map_lanes_frenet_m[lane_id].ToFrenet(p_xy, p_sd);
      if (abs(p_sd.d) < min_d) {
        min_d = abs(p_sd.d);
        matched_lane = lane_id;
      }
    }

    if (min_d > 1.75 || matched_lane == -1) continue; // far from any lanes, ignore

    // predict with lane center line
    casper_auto_msgs::Prediction msg_pred;
    msg_pred.dt = float(dt_m);
    msg_pred.lane_id = matched_lane;
    msg_pred.agent_id = veh.id;
    msg_pred.length = veh.shape.dimensions[0];
    msg_pred.width = veh.shape.dimensions[1];

    casper_auto_msgs::IntentionTrajectory int_traj;
    int_traj.trajectory_probability = 1.0;

    double v = std::min(std::hypot(veh.twist.linear.x,
                                   veh.twist.linear.y),
                        max_speed_agts_m);

    geometry::Point_Frenet p_sd;
    map_lanes_frenet_m[matched_lane].ToFrenet(p_xy, p_sd);

    int_traj.trajectory_estimated.waypoints.resize(np_m);
    int_traj.trajectory_uncertainty.waypoints.resize(np_m);
    uint32_t interp_back_path = 20;  // Interpolate laterally back to path
    for (uint32_t t = 0; t < np_m; t++) {
      // Position
      geometry::Point2D pred_xy;
      double road_dir;
      if (t < interp_back_path) {
        double d_val = p_sd.d - double(t) * p_sd.d / double(interp_back_path);
        map_lanes_frenet_m[matched_lane].ToCartesian(
            geometry::Point_Frenet(p_sd.s + v * double(dt_m) * double(t),
                                   d_val),
            pred_xy, road_dir);
      } else {
        map_lanes_frenet_m[matched_lane].ToCartesian(
            geometry::Point_Frenet(p_sd.s + v * double(dt_m) * double(t), 0.0),
            pred_xy, road_dir);
      }

      geometry::Point2D pred_out = globalToLocal(ego_xy_m, ego_theta_m, pred_xy);

      tf::Quaternion q;
      q.setRPY(0.0, 0.0, road_dir);

      int_traj.trajectory_estimated.waypoints[t].pose.pose.position.x =
          pred_out.x;
      int_traj.trajectory_estimated.waypoints[t].pose.pose.position.y =
          pred_out.y;
      int_traj.trajectory_estimated.waypoints[t].pose.pose.orientation.x =
          q.getX();
      int_traj.trajectory_estimated.waypoints[t].pose.pose.orientation.y =
          q.getY();
      int_traj.trajectory_estimated.waypoints[t].pose.pose.orientation.z =
          q.getZ();
      int_traj.trajectory_estimated.waypoints[t].pose.pose.orientation.w =
          q.getW();

      // Velocity
      int_traj.trajectory_estimated.waypoints[t].twist.twist = veh.twist;

      // Uncertainty Position
      int_traj.trajectory_uncertainty.waypoints[t].pose.pose.position.x = 0.0;
      int_traj.trajectory_uncertainty.waypoints[t].pose.pose.position.y = 0.0;

      // Uncertainty Velocity
      int_traj.trajectory_uncertainty.waypoints[t].twist.twist.linear.x = 0.0;
      int_traj.trajectory_uncertainty.waypoints[t].twist.twist.linear.y = 0.0;
    }

    msg_pred.trajectories.push_back(int_traj);
    vehicles_prediction_msg.predictions.push_back(msg_pred);
  }
}

void AgentsEstimation::PredictTrajectoryPedestrians(
    casper_auto_msgs::PredictionArray &pedestrians_prediction_msg) {
  uint64_t n_ped = pedestrian_vec_m.size();
  // Constant velocity for each pedestrian
  for (uint32_t k = 0; k < n_ped; k++) {
    uint32_t ped_id = pedestrian_vec_m[k].id;

    // Save information in the message
    casper_auto_msgs::Prediction msg_pred;
    msg_pred.dt = float(dt_m);
    msg_pred.agent_id = ped_id;
    msg_pred.length = 0.5;
    msg_pred.width = 0.5;
    casper_auto_msgs::IntentionTrajectory int_traj;
    int_traj.trajectory_probability = 1.0;

    double x, y, v, roll, pitch, yaw;
    x = pedestrian_vec_m[k].pose.position.x;
    y = pedestrian_vec_m[k].pose.position.y;
    v = std::min(std::hypot(pedestrian_vec_m[k].twist.linear.x,
                            pedestrian_vec_m[k].twist.linear.y),
                 max_speed_agts_m);
    tf::Quaternion q(pedestrian_vec_m[k].pose.orientation.x,
                     pedestrian_vec_m[k].pose.orientation.y,
                     pedestrian_vec_m[k].pose.orientation.z,
                     pedestrian_vec_m[k].pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    int_traj.trajectory_estimated.waypoints.resize(np_m);
    int_traj.trajectory_uncertainty.waypoints.resize(np_m);
    for (uint32_t t = 0; t < np_m; t++) {
      // Position
      int_traj.trajectory_estimated.waypoints[t].pose.pose.position.x =
          x + v * double(dt_m) * cos(yaw) * double(t);
      int_traj.trajectory_estimated.waypoints[t].pose.pose.position.y =
          y + v * double(dt_m) * sin(yaw) * double(t);
      int_traj.trajectory_estimated.waypoints[t].pose.pose.orientation =
          pedestrian_vec_m[k].pose.orientation;

      // Velocity
      int_traj.trajectory_estimated.waypoints[t].twist.twist =
          pedestrian_vec_m[k].twist;

      // Uncertainty Position
      int_traj.trajectory_uncertainty.waypoints[t].pose.pose.position.x = 0.0;
      int_traj.trajectory_uncertainty.waypoints[t].pose.pose.position.y = 0.0;

      // Uncertainty Velocity
      int_traj.trajectory_uncertainty.waypoints[t].twist.twist.linear.x = 0.0;
      int_traj.trajectory_uncertainty.waypoints[t].twist.twist.linear.y = 0.0;
    }

    msg_pred.trajectories.push_back(int_traj);
    pedestrians_prediction_msg.predictions.push_back(msg_pred);
  }
}
