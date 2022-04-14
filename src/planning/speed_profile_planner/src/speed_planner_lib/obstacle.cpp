/**
 * @file obstacle.cpp
 * @brief obstacle
 **/

#include "speed_planner_lib/obstacle.h"

namespace planning {

Obstacle::Obstacle(ObstacleType obs_type,
                   std::vector<TrajectoryPoint> trajectory, bool in_target_lane)
    : obs_type_(obs_type), predicted_trajectory_(trajectory),
      in_target_lane_(in_target_lane) {

  AdjustSize();

  DEBUG_MSG("Obstacle Type:" << obs_type);

  if (!trajectory.empty()) {
    DEBUG_MSG("Initial pos: x = " << trajectory[0].x
                                  << ", y = " << trajectory[0].y
                                  << ", v = " << trajectory[0].v);
    init_pos_ = Point2D(trajectory[0].x, trajectory[0].y);
    speed_ = trajectory[0].v;
    for (int i = 0; i < trajectory.size(); i++) {
      Point2D curr(trajectory[i].x, trajectory[i].y);
      reference_path_.push_back(curr);
    }
    // convert to Frenet frame
    ref_path_frenet_.SetPath(reference_path_);
    // for computing ego sd
    int init_idx = closestPoint(reference_path_, init_pos_.x, init_pos_.y);
    Point_Frenet init_sd;
    ref_path_frenet_.ToFrenet(init_pos_, init_idx, init_sd);
    init_s_ = init_sd.s + 0.0001;
  }
}

void Obstacle::AdjustSize() {
  switch (obs_type_) {
  case CAR:
    width_ = kCarWidth;
    length_ = kCarLength;
    break;
  case TRUCK:
    width_ = kTruckWidth;
    length_ = kTruckLength;
    break;
  case BICYCLE:
    width_ = kBycicleWidth;
    length_ = kBycicleLength;
    break;
  case PEDESTRIAN:
    width_ = kPedestrianWidth;
    length_ = kPedestrianLength;
    break;
  case STOPSIGN:
    width_ = kStopsignWidth;
    length_ = kStopsignLength;
    break;
  case CROSSWALK:
    width_ = kCrosswalkWidth;
    length_ = kCrosswalkLength;
    break;
  case TRAFFICLIGHT:
    width_ = kTFLightWidth;
    length_ = kTFLightLength;
    break;
  default:
    width_ = 0.5;
    length_ = 0.5;
    break;
  }
}

TrajectoryPoint Obstacle::GetTrajectoryPointAtTime(double relative_time) {
  double s = init_s_ + speed_ * relative_time;
  Point_Frenet p_sd(s, 0);
  Point2D p_xy;
  ref_path_frenet_.ToCartesian(p_sd, p_xy);
  // DEBUG_MSG("Frenet: " << s << ", " << p_sd.d << ", Cartisian: " << p_xy.x <<
  // ", " << p_xy.y);
  return TrajectoryPoint(p_xy.x, p_xy.y, speed_, relative_time);
}

std::vector<double> Obstacle::SplitTimeline(double time_range,
                                            double unit_time) {
  std::vector<double> pred_time;
  unit_time = std::max(unit_time, kMinUnitTime);
  double curr_time = 0.0;
  while (curr_time < time_range) {
    pred_time.push_back(curr_time);
    curr_time += unit_time;
  }
  pred_time.push_back(time_range);

  // std::cout << "Splitted Time:" << std::endl;
  // for(double time : pred_time) {
  //   std::cout << time << std::endl;
  // }

  return pred_time;
}

std::vector<TrajectoryPoint>
Obstacle::PredictTrajectoryOverTime(double time_range, double unit_time) {
  std::vector<TrajectoryPoint> pred_traj;
  std::vector<double> pred_time = SplitTimeline(time_range, unit_time);
  if (speed_ < 0.01) {
    for (int i = 0; i < pred_time.size(); i++) {
      TrajectoryPoint traj_point =
          TrajectoryPoint(init_pos_.x, init_pos_.y, speed_, pred_time[i]);
      pred_traj.push_back(traj_point);
    }
  } else {
    for (int i = 0; i < pred_time.size(); i++) {
      TrajectoryPoint traj_point = GetTrajectoryPointAtTime(pred_time[i]);
      pred_traj.push_back(traj_point);
    }
  }
  return pred_traj;
}

} // namespace planning
