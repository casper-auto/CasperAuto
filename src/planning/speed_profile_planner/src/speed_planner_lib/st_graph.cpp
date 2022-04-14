/**
 * @file st_graph.cpp
 * @brief StGraph class definition
 **/

#include "speed_planner_lib/st_graph.h"

namespace planning {

STGraph::STGraph() {}

STGraph::STGraph(std::vector<Point2D> reference_path,
                 std::vector<Obstacle> obstacles, double s_range,
                 double t_range, double unit_time, double stop_line_offset,
                 double car_following_distance, double oncoming_car_distance)
    : reference_path_(reference_path), obstacles_(obstacles), s_range_(s_range),
      t_range_(t_range), unit_time_(unit_time),
      stop_line_offset_(stop_line_offset),
      car_following_distance_(car_following_distance),
      oncoming_car_distance_(oncoming_car_distance) {
  ego_path_frenet_ = Frenet(reference_path);
  ComputeAllOccupied();
}

void STGraph::ComputeAllOccupied() {
  occupied_areas_.resize(0);
  for (int i = 0; i < obstacles_.size(); i++) {
    STArea area = ComputeOccupiedByObstacle(obstacles_[i]);
    occupied_areas_.push_back(area);
  }

  if (false) {
    // Clear previous plot
    plt::clf();

    plt::xlim(0, 8);
    plt::ylim(0, 50);

    std::vector<double> xticks;
    for (int i = 0; i <= 8; i++) {
      xticks.push_back(i);
    }

    std::vector<double> yticks;
    for (int i = 0; i <= 50; i += 5) {
      yticks.push_back(i);
    }

    plt::xticks(xticks);
    plt::yticks(yticks);

    for (auto area : occupied_areas_) {
      std::vector<double> t, s1, s2;

      if (area.cut_in.t >= area.cut_out.t)
        continue;

      t = {area.cut_in.t, area.cut_out.t};
      s1 = {area.cut_in.s - area.lower_bound,
            area.cut_out.s - area.lower_bound};
      s2 = {area.cut_in.s + area.upper_bound,
            area.cut_out.s + area.upper_bound};

      std::map<std::string, std::string> keywords;
      keywords["alpha"] = "0.4";
      keywords["color"] = "red";
      // keywords["hatch"] = "-";

      plt::fill_between(t, s1, s2, keywords);
    }

    // Add graph title
    plt::title("ST-Graph");
    plt::xlabel("Time (s)");
    plt::ylabel("Distance along the road (m)");

    // Enable legend.
    plt::legend();
    // Enable grid.
    plt::grid();
    // Display plot continuously
    plt::pause(0.01);
  }
}

// core method
STArea STGraph::ComputeOccupiedByObstacle(Obstacle obstacle) {
  STArea area;
  std::vector<TrajectoryPoint> predicted_traj =
      obstacle.PredictTrajectoryOverTime(t_range_, unit_time_);
  DEBUG_MSG("  Size of predicted traj: " << predicted_traj.size());
  STPoint cut_in = CutIn(predicted_traj);
  STPoint cut_out = CutOut(predicted_traj);
  double obs_length = 0;
  if (obstacle.GetType() == ObstacleType::STOPSIGN) {
    obs_length = obstacle.GetLength();
    area = STArea(cut_in, cut_out, obs_length / 2.0 + stop_line_offset_,
                  0.0 + stop_line_offset_);
  } else if (obstacle.GetType() == ObstacleType::CROSSWALK) {
    obs_length = obstacle.GetLength();
    area = STArea(cut_in, cut_out, obs_length / 2.0, obs_length / 2.0);
  } else if (obstacle.GetType() == ObstacleType::CAR &&
             obstacle.IsInTargetLane()) {
    obs_length = car_following_distance_;
    area = STArea(cut_in, cut_out, obs_length / 2.0, obs_length / 2.0);
  } else if (obstacle.GetType() == ObstacleType::CAR &&
             !obstacle.IsInTargetLane()) {
    obs_length = oncoming_car_distance_;
    area = STArea(cut_in, cut_out, obs_length / 2.0, obs_length / 2.0);
  } else {
    obs_length = obstacle.GetLength() * 2;
    area = STArea(cut_in, cut_out, obs_length / 2.0, obs_length / 2.0);
  }
  DEBUG_MSG("  Obstacle " << obstacle.GetType() << " IsInTargetLane: "
                          << obstacle.IsInTargetLane() << " Occupied Area: "
                          << area.cut_in.s - area.lower_bound << ", "
                          << area.cut_in.t << "; "
                          << area.cut_out.s + area.lower_bound << ", "
                          << area.cut_out.t);
  return area;
}

STPoint STGraph::CutIn(std::vector<TrajectoryPoint> predicted_traj) {
  double s_in = s_range_;
  double t_in = t_range_;
  for (int i = 0; i < predicted_traj.size(); i++) {
    double x = predicted_traj[i].x;
    double y = predicted_traj[i].y;
    Point2D p(x, y);
    Vector2f pos(x, y);
    Vector2f dir;
    if (i == 0) {
      dir.x = predicted_traj[1].x - predicted_traj[0].x;
      dir.y = predicted_traj[1].y - predicted_traj[0].y;
    } else {
      dir.x = predicted_traj[i].x - predicted_traj[i - 1].x;
      dir.y = predicted_traj[i].y - predicted_traj[i - 1].y;
    }
    for (int j = 0; j < reference_path_.size(); j++) {
      double distance = p.DistanceTo(reference_path_[j]);
      double radius =
          sqrt(pow(kEgoCarLength / 2, 2) + pow(kEgoCarWidth / 2, 2));

      Vector2f other_pos(reference_path_[j].x, reference_path_[j].y);
      Vector2f other_dir;
      if (j == 0) {
        other_dir.x = reference_path_[1].x - reference_path_[0].x;
        other_dir.y = reference_path_[1].y - reference_path_[0].y;
      } else {
        other_dir.x = reference_path_[j].x - reference_path_[j - 1].x;
        other_dir.y = reference_path_[j].y - reference_path_[j - 1].y;
      }
      bool collide = (distance < kEgoCarLength / 2);
      // || CollisionChecker::hasCollision(pos, dir, kEgoCarLength,
      // kEgoCarWidth,
      //                                   other_pos, other_dir, kEgoCarLength,
      //                                   kEgoCarWidth);
      if (collide) {
        // compute s in frenet frame
        int idx = closestPoint(reference_path_, predicted_traj[i].x,
                               predicted_traj[i].y);
        Point_Frenet sd;
        ego_path_frenet_.ToFrenet({predicted_traj[i].x, predicted_traj[i].y},
                                  idx, sd);

        s_in = sd.s - kEgoCarLength / 2.0;
        t_in = predicted_traj[i].relative_time;
        return STPoint(s_in, t_in);
      }
    }
  }
  return STPoint(s_in, t_in);
}

STPoint STGraph::CutOut(std::vector<TrajectoryPoint> predicted_traj) {
  double s_out = 0.0;
  double t_out = 0.0;
  for (int i = predicted_traj.size() - 1; i >= 0; i--) {
    double x = predicted_traj[i].x;
    double y = predicted_traj[i].y;
    Point2D p(x, y);
    Vector2f pos(x, y);
    Vector2f dir;
    if (i == 0) {
      dir.x = predicted_traj[1].x - predicted_traj[0].x;
      dir.y = predicted_traj[1].y - predicted_traj[0].y;
    } else {
      dir.x = predicted_traj[i].x - predicted_traj[i - 1].x;
      dir.y = predicted_traj[i].y - predicted_traj[i - 1].y;
    }
    for (int j = 0; j < reference_path_.size(); j++) {
      double distance = p.DistanceTo(reference_path_[j]);
      double radius =
          sqrt(pow(kEgoCarLength / 2, 2) + pow(kEgoCarWidth / 2, 2));

      Vector2f other_pos(reference_path_[j].x, reference_path_[j].y);
      Vector2f other_dir;
      if (j == 0) {
        other_dir.x = reference_path_[1].x - reference_path_[0].x;
        other_dir.y = reference_path_[1].y - reference_path_[0].y;
      } else {
        other_dir.x = reference_path_[j].x - reference_path_[j - 1].x;
        other_dir.y = reference_path_[j].y - reference_path_[j - 1].y;
      }
      bool collide = (distance < kEgoCarLength / 2);
      // ||  CollisionChecker::hasCollision(pos, dir, kEgoCarLength,
      // kEgoCarWidth,
      //                                    other_pos, other_dir, kEgoCarLength,
      //                                    kEgoCarWidth);
      if (collide) {
        // compute s in frenet frame
        int idx = closestPoint(reference_path_, predicted_traj[i].x,
                               predicted_traj[i].y);
        Point_Frenet sd;
        ego_path_frenet_.ToFrenet({predicted_traj[i].x, predicted_traj[i].y},
                                  idx, sd);

        s_out = sd.s - kEgoCarLength / 2.0;
        t_out = predicted_traj[i].relative_time;
        return STPoint(s_out, t_out);
      }
    }
  }
  return STPoint(s_out, t_out);
}

} // namespace planning
