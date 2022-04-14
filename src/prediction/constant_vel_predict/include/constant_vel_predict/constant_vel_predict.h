#ifndef CONSTANT_VEL_PREDICT_H
#define CONSTANT_VEL_PREDICT_H

#include <cmath>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <casper_auto_msgs/CenterLanes.h>
#include <casper_auto_msgs/PredictionArray.h>
#include <casper_auto_msgs/VehicleState.h>
#include <derived_object_msgs/Object.h>
#include <derived_object_msgs/ObjectArray.h>

#include <constant_vel_predict/frenet_frame.h>

struct VehiclesLaneData {
  std::vector<casper_auto_msgs::VehicleState> sorted_vehicles;
  int32_t id;
};

////////////////////////////////////////////////////////////////////////////////
// Agents estimation class
////////////////////////////////////////////////////////////////////////////////

class AgentsEstimation {
 public:
  AgentsEstimation();

 private:
  // Simulation variables
  float plan_t_m;
  float dt_m;
  uint32_t np_m;
  double max_speed_agts_m;

  // Vehicles variables
  geometry::Point2D ego_xy_m;
  double ego_theta_m;
  std::vector<VehiclesLaneData> vehicles_lanes_m;
  std::vector<derived_object_msgs::Object> vehicle_vec_m;
  std::vector<derived_object_msgs::Object> pedestrian_vec_m;
  std::unordered_map<int32_t, geometry::Frenet> map_lanes_frenet_m;
  std::unordered_map<uint32_t, std::vector<int32_t>> map_car_lanes_m;

  // Warning and times for each message
  ros::Time prev_t_objects_m;
  ros::Time prev_t_err_objects_m;
  ros::Time prev_t_err_lanes_m;

  // ROS variables
  ros::NodeHandle nh_m;

  ros::Subscriber ego_pose_sub_m;
  ros::Subscriber detected_objects_sub_m;
  ros::Subscriber reference_lanes_sub_m;

  ros::Publisher vehicles_prediction_pub_m;
  ros::Publisher pedestrians_prediction_pub_m;

  ros::Timer timer_m;

  // ROS callbacks
  void EgoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void DetectedObjectsCallback(
      const derived_object_msgs::ObjectArray::ConstPtr &msg);
  void ReferenceLanesCallback(const casper_auto_msgs::CenterLanes::ConstPtr &msg);
  void TimerCallback(const ros::TimerEvent &);

  // Predict trajectory
  void PredictTrajectoryVehicles(
      casper_auto_msgs::PredictionArray &vehicles_prediction_msg);
  void PredictTrajectoryPedestrians(
      casper_auto_msgs::PredictionArray &msg_pedestrians);
};

#endif /* CONSTANT_VEL_PREDICT_H*/
