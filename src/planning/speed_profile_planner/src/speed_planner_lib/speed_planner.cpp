/**
 * @file speed_planner.cpp
 * @brief SpeedPlanner class definition.
 **/

#include "speed_planner_lib/speed_planner.h"

namespace planning {

SpeedPlanner::SpeedPlanner()
    : ego_vehicle_(new Vehicle()), optimizer_(ego_vehicle_), left_hand_(false),
      low_speed_cut_(false), low_speed_thresh_(0.5), velocity_offset_(0),
      car_following_distance_(10), oncoming_car_distance_(15),
      stop_line_offset_(0.0), release_time_(3.0), crosswalk_span_(2.0),
      crosswalk_shift_(0.0), visualize_(0) {
  DEBUG_MSG("SpeedPlanner::constructor with default config");
}

SpeedPlanner::SpeedPlanner(MPCConfig *mpc_config, bool left_hand,
                           bool low_speed_cut, double low_speed_thresh,
                           int velocity_offset, double car_following_distance,
                           double oncoming_car_distance,
                           double stop_line_offset, double release_time,
                           double crosswalk_span, double crosswalk_shift,
                           int viz_mode)
    : ego_vehicle_(new Vehicle()), optimizer_(ego_vehicle_, mpc_config),
      left_hand_(left_hand), low_speed_cut_(low_speed_cut),
      low_speed_thresh_(low_speed_thresh), velocity_offset_(velocity_offset),
      car_following_distance_(car_following_distance),
      oncoming_car_distance_(oncoming_car_distance),
      stop_line_offset_(stop_line_offset), release_time_(release_time),
      crosswalk_span_(crosswalk_span), crosswalk_shift_(crosswalk_shift),
      visualize_(viz_mode) {
  DEBUG_MSG("SpeedPlanner::constructor with input config.");
}

SpeedPlanner::~SpeedPlanner() {
  delete ego_vehicle_;
  ego_vehicle_ = NULL;
}

Status SpeedPlanner::ReloadOptimizerConfig(MPCConfig *mpc_config) {
  optimizer_ = STOptimizer(ego_vehicle_, mpc_config);
  return Status::OK();
}

Status SpeedPlanner::Plan(const std::vector<Obstacle> &perception_obstacles,
                          std::vector<TrajectoryPoint> &reference_trajectory) {
  DEBUG_MSG(
      "\n****************************************************************");
  DEBUG_MSG("[Info] Speed Planner Loop.");
  DEBUG_MSG("****************************************************************");

  if (reference_trajectory.size() == 0) {
    return Status("Warning: ",
                  "Invalid input: 'reference_trajectory' is empty");
  }

  /**************************************************
   * Stage 1: Prepare data.
   **************************************************/
  DEBUG_MSG("\n[Info] Stage 1: Prepare data.");

  // Set current configuration of vehicle
  ego_vehicle_->SetCurrentConfig(reference_trajectory[0].x,
                                 reference_trajectory[0].y,
                                 reference_trajectory[0].v);
  ego_vehicle_->PrintVehicleConfiguration();

  std::vector<Point2D> reference_path;

  // TODO: Code optimization.
  // Can be used as ref_trjectory instead of converting it to path
  for (auto e : reference_trajectory) {
    double x = e.x;
    double y = e.y;
    reference_path.push_back(Point2D(x, y));
  }

  ego_path_ = reference_path;

  DEBUG_MSG("The size of reference_path: " << reference_path.size());

  /**************************************************
   * Stage 2: Process Obstacles
   **************************************************/
  DEBUG_MSG("\n[Info] Stage 2: Process Obstacles into ST representations.");

  // filter out insignificant obstacles
  std::vector<Obstacle> filtered_obstacles;

  ProcessStopSigns(filtered_obstacles);

  // ProcessCrosswalks(filtered_obstacles);

  // ProcessTrafficLights(filtered_obstacles);

  // DEBUG_MSG("Process Static Obstacles done!" << "num = " <<
  // filtered_obstacles.size());

  /**
   * Process Dynamic Obstacles
   **/
  ProcessOtherCars(perception_obstacles, filtered_obstacles);

  // ProcessPedestrians(perception_obstacles,
  //                    filtered_obstacles,
  //                    kPlannedTRange,
  //                    kPlannedUnitTime);

  DEBUG_MSG("ObstacleFilter done!"
            << "num = " << filtered_obstacles.size());

  /**************************************************
   * Stage 3: Populate the ST-Ggraph with obstacles
   **************************************************/
  DEBUG_MSG("\n[Info] Stage 3: Populate the ST-Ggraph.");

  STGraph st_graph(reference_path, filtered_obstacles, kPlannedSRange,
                   kPlannedTRange, kPlannedUnitTime, stop_line_offset_,
                   car_following_distance_, oncoming_car_distance_);

  std::vector<STArea> occupied_areas = st_graph.GetAllOccupied();

  ProcessPedestrians(perception_obstacles, filtered_obstacles, kPlannedTRange,
                     kPlannedUnitTime, occupied_areas);

  /**************************************************
   * Stage 4: Populate the ST-Ggraph with obstacles
   **************************************************/
  DEBUG_MSG("\n[Info] Stage 4: Compute a rough viable paths in ST-Graph.");

  // Generate all candidate plans represented by connected s-t cells
  STCellPlanner st_cell_planner(occupied_areas, kPlannedSRange, kPlannedTRange,
                                kPlannedUnitTime);

  std::vector<std::vector<STCell>> candidate_plans =
      st_cell_planner.SearchCandidatePlans();

  DEBUG_MSG("The number of solutions: " << candidate_plans.size());
  for (int i = 0; i < candidate_plans.size(); i++) {
    DEBUG_MSG("solution : " << i);
    for (int j = 0; j < candidate_plans[i].size(); j++) {
      DEBUG_MSG("[index = " << j << "]: " << candidate_plans[i][j].min_s << ", "
                            << candidate_plans[i][j].max_s);
    }
  }

  /**************************************************
   * Stage 5: ST Path optimization.
   **************************************************/
  DEBUG_MSG("\n[Info] Stage 5: ST Path optimization.");

  // pick the best coridor and optimize the path
  std::vector<STConfig> desired_trajectory;

  Point2D ego_pos = ego_vehicle_->GetPosition();
  double ego_x = ego_pos.x;
  double ego_y = ego_pos.y;

  // fill v value in reference_trajectory
  double s = 0.0;
  double ds = 0.0;

  STConfig st_config;
  st_config.s = s;
  st_config.s_dot = ego_vehicle_->GetVelocity();
  st_config.s_dotdot = 0.0;
  st_config.s_dot_limit = reference_trajectory[0].v_limit;
  desired_trajectory.push_back(st_config);
  for (unsigned int i = 1; i < reference_trajectory.size(); i++) {
    double dx = reference_trajectory[i].x - reference_trajectory[i - 1].x;
    double dy = reference_trajectory[i].y - reference_trajectory[i - 1].y;
    ds = sqrt(dx * dx + dy * dy);
    s += ds;
    if (s > kPlannedSRange)
      break;
    STConfig st_config;
    st_config.s = s;
    st_config.s_dot = 0.0;
    st_config.s_dotdot = 0.0;
    st_config.s_dot_limit = reference_trajectory[i].v_limit;
    desired_trajectory.push_back(st_config);
  }

  Status op_s = optimizer_.optimizePaths(candidate_plans, desired_trajectory);

  // create an spline of the s vs s_dot for interpolation
  tk::spline s_s_dot_sp;

  std::vector<double> s_val;
  std::vector<double> s_dot_val;

  unsigned int nbPointToBeReported = kPlannedTimeHorizon / kPlannedUnitTime;
  unsigned int size_desired_trajectory = desired_trajectory.size();
  int nbsteps = std::min(nbPointToBeReported, size_desired_trajectory);
  if (op_s == Status::OK()) {
    for (unsigned int i = 0; i < nbsteps; i++) {
      s_val.push_back(desired_trajectory[i].s);
      s_dot_val.push_back(desired_trajectory[i].s_dot);
    }
  } else {
    double s = 0.0;
    for (unsigned int i = 0; i < nbPointToBeReported; i++) {
      s_val.push_back(s);
      s_dot_val.push_back(0.0);
      s += 0.5;
    }
  }

  DEBUG_MSG("********************************");
  DEBUG_MSG("s_val: ");
  for (auto s : s_val) {
    DEBUG_MSG(s << ", ");
  }
  DEBUG_MSG("********************************");

  DEBUG_MSG("********************************");
  DEBUG_MSG("s_dot_val: ");
  for (auto s : s_dot_val) {
    DEBUG_MSG(s << ", ");
  }
  DEBUG_MSG("********************************");

  //****************************************************************************
  // Visualize ST Graph
  //****************************************************************************
  if (visualize_ == 1 || visualize_ == 2) {
    plt::figure(1);

    // Clear previous plot
    plt::clf();

    plt::xlim(0, 7);
    plt::ylim(0, 50);

    std::vector<double> xticks;
    for (int i = 0; i <= 7; i++) {
      xticks.push_back(i);
    }

    std::vector<double> yticks;
    for (int i = 0; i <= 50; i += 5) {
      yticks.push_back(i);
    }

    plt::xticks(xticks);
    plt::yticks(yticks);

    for (auto area : occupied_areas) {
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

    for (int i = 0; i < candidate_plans.size(); i++) {
      std::vector<STCell> plan = candidate_plans[i];
      std::vector<double> t, s1, s2;
      for (int j = 0; j < plan.size(); j++) {
        t.push_back(j);
        s1.push_back(plan[j].min_s);
        s2.push_back(plan[j].max_s);
      }

      std::map<std::string, std::string> keywords;
      keywords["alpha"] = "0.4";
      keywords["color"] = "blue";
      // keywords["hatch"] = "-";

      plt::fill_between(t, s1, s2, keywords);
    }

    std::vector<double> t, s;

    for (int i = 0; i < desired_trajectory.size(); i++) {
      t.push_back(i);
      s.push_back(desired_trajectory[i].s);
    }

    plt::plot(t, s, "g-");

    // Add graph title
    plt::title("ST-Graph");
    plt::xlabel("Time (s)");
    plt::ylabel("Distance along the road (m)");

    // Enable grid.
    plt::grid();
    // Display plot continuously
    plt::pause(0.01);
  }

  //****************************************************************************
  // Visualize Speed Variance
  //****************************************************************************
  if (visualize_ == 2) {

    if (act_vel_history_.size() < 100) {
      act_vel_history_.push_back(ego_vehicle_->GetVelocity());
    } else {
      act_vel_history_.push_back(ego_vehicle_->GetVelocity());
      act_vel_history_.erase(act_vel_history_.begin());

      plt::figure(2);

      // Clear previous plot
      plt::clf();

      plt::xlim(0, 10);
      plt::ylim(0, 5);

      std::vector<double> t, v;

      for (int i = 0; i < act_vel_history_.size(); i++) {
        t.push_back(i * 0.1);
        v.push_back(act_vel_history_[i]);
      }

      plt::plot(t, v, "g-");

      // Add graph title
      plt::title("Velocity Variance");
      plt::xlabel("Time (s)");
      plt::ylabel("Velocity (m/s)");

      // Enable grid.
      plt::grid();
      // Display plot continuously
      plt::pause(0.01);
    }
  }

  /**************************************************
   * Stage 6: Post processing for the command velocity.
   **************************************************/
  DEBUG_MSG("\n[Info] Stage 6: Post processing for the command velocity.");

  s_s_dot_sp.set_points(s_val, s_dot_val);

  double s_max = 0.0;
  if (s_val.size() > 0){
       s_max = s_val[s_val.size()-1];
  }

  // fill v value in reference_trajectory
  std::cout << "UPDATE: ";
  double s_acc = 0.0;
  for (unsigned int i = 1; i < reference_trajectory.size(); i++) {
    double dx = reference_trajectory[i].x - reference_trajectory[i - 1].x;
    double dy = reference_trajectory[i].y - reference_trajectory[i - 1].y;
    double ds = sqrt(dx * dx + dy * dy);
    s_acc += ds;
    //std::cout << "[ " << s_acc << " , " << s_s_dot_sp(s_acc) << "] , ";
    if (s_acc < s_max){
        double vv = std::max(0.0, s_s_dot_sp(s_acc));
        vv = (vv > 0.1)? vv: 0.0;
        reference_trajectory[i].v = std::min(reference_trajectory[i].v_limit, vv);
    }else{
        double vv = std::max(0.0, s_s_dot_sp(s_max));
        vv = (vv > 0.1)? vv: 0.0;
        reference_trajectory[i].v = std::min(reference_trajectory[i].v_limit, vv);
    }
  }

  cmd_vels_.resize(0);

  if (op_s == Status::OK()) {
    // up to 20 commands considering the planning timestep in mpc is 1sec
    for (int i = 1; i < 21; i++) {
      double t = 0.01 * (i);
      double val = desired_trajectory[0].s_dot
                   + desired_trajectory[0].s_dotdot * t
                   + 0.5 * desired_trajectory[0].s_dotdotdot * t * t;
      cmd_vels_.push_back(std::min(0.9 * reference_trajectory[0].v_limit,std::max(0.0, val)));
    }
  }

  DEBUG_MSG("********************************");
  DEBUG_MSG("e.v: ");
  for (auto e : reference_trajectory) {
    DEBUG_MSG(e.v << ", ");
  }
  DEBUG_MSG("********************************");

  DEBUG_MSG("********************************");
  DEBUG_MSG("v: ");
  for (auto v : cmd_vels_) {
    DEBUG_MSG(v << ", ");
  }
  DEBUG_MSG("********************************");

  return Status::OK();
}

std::vector<Point2D> SpeedPlanner::GetRoadLineByPoint(Point2D point) {
  return GetPathByPoint(point, lane_lines_);
}

std::vector<Point2D> SpeedPlanner::GetWalkLineByPoint(Point2D point) {
  return GetPathByPoint(point, side_walks_);
}

Status SpeedPlanner::ParseLocalMap(std::string filename) {
  // Create empty property tree object
  pt::ptree tree;

  // Parse the XML into the property tree.
  pt::read_xml(filename, tree);

  // Use the throwing version of get to find the debug filename.
  // If the path cannot be resolved, an exception is thrown.
  std::string id = tree.get<std::string>("scenario.id");
  DEBUG_MSG("scenario id = " << id);

  // Use the default-value version of get to find the debug level.
  // Note that the default value is used to deduce the target type.
  // int m_level = tree.get("debug.level", 0);

  // Use get_child to find the node containing the modules, and iterate over
  // its children. If the path cannot be resolved, get_child throws.
  // A C++11 for-range loop would also work.
  BOOST_FOREACH (pt::ptree::value_type &v, tree.get_child("scenario")) {
    // The data function is used to access the data stored in a node.
    if (v.first == "traffic_light") {
      DEBUG_MSG("traffic_light");
      std::vector<Point2D> traffic_light;
      BOOST_FOREACH (pt::ptree::value_type &vv, v.second) {
        if (vv.first == "point") {
          DEBUG_MSG("  point:");
          DEBUG_MSG("    key = " << vv.first);
          DEBUG_MSG("    x = " << vv.second.get<std::string>("x"));
          DEBUG_MSG("    y = " << vv.second.get<std::string>("y"));
          double x = std::stod(vv.second.get<std::string>("x"));
          double y = std::stod(vv.second.get<std::string>("y"));
          Point2D point(x, y);
          traffic_light.push_back(point);
        }
      }
      traffic_lights_.push_back(traffic_light);
    }

    if (v.first == "stop_sign") {
      DEBUG_MSG("stop_sign");
      std::vector<Point2D> stop_line;
      double release_time;
      BOOST_FOREACH (pt::ptree::value_type &vv, v.second) {
        if (vv.first == "point") {
          DEBUG_MSG("  point:");
          DEBUG_MSG("    key = " << vv.first);
          DEBUG_MSG("    x = " << vv.second.get<std::string>("x"));
          DEBUG_MSG("    y = " << vv.second.get<std::string>("y"));
          double x = std::stod(vv.second.get<std::string>("x"));
          double y = std::stod(vv.second.get<std::string>("y"));
          Point2D point(x, y);
          stop_line.push_back(point);
        } else if (vv.first == "release_time") {
          DEBUG_MSG("  release time = " << vv.second.data() << " sec");
          release_time = std::stod(vv.second.data());
        }
      }
      stop_signs_.push_back(stop_line);
      stop_release_time_.push_back(release_time);
    }

    if (v.first == "crosswalk") {
      DEBUG_MSG("crosswalk");
      std::vector<Point2D> crosswalk;
      double crosswalk_span;
      BOOST_FOREACH (pt::ptree::value_type &vv, v.second) {
        if (vv.first == "point") {
          DEBUG_MSG("  point:");
          DEBUG_MSG("    key = " << vv.first);
          DEBUG_MSG("    x = " << vv.second.get<std::string>("x"));
          DEBUG_MSG("    y = " << vv.second.get<std::string>("y"));
          double x = std::stod(vv.second.get<std::string>("x"));
          double y = std::stod(vv.second.get<std::string>("y"));
          Point2D point(x, y);
          crosswalk.push_back(point);
        } else if (vv.first == "span") {
          DEBUG_MSG("  crosswalk span = " << vv.second.data() << " m");
          crosswalk_span = std::stod(vv.second.data());
        }
      }
      crosswalks_.push_back(crosswalk);
      crosswalk_spans_.push_back(crosswalk_span);
    }

    if (v.first == "ego_reference_path") {
      DEBUG_MSG("ego_reference_path");
      BOOST_FOREACH (pt::ptree::value_type &vv, v.second) {
        if (vv.first == "point") {
          DEBUG_MSG("  point:");
          DEBUG_MSG("    key = " << vv.first);
          DEBUG_MSG("    x = " << vv.second.get<std::string>("x"));
          DEBUG_MSG("    y = " << vv.second.get<std::string>("y"));
          double x = std::stod(vv.second.get<std::string>("x"));
          double y = std::stod(vv.second.get<std::string>("y"));
          double speed_limit =
              std::stod(vv.second.get<std::string>("speed_limit"));
          Point2D point(x, y);
          TrajectoryPoint traj_point(x, y, 0.0, 0.0, speed_limit);
          ego_path_.push_back(point);
          ego_traj_.push_back(traj_point);
        }
      }
    }

    if (v.first == "target_path") {
      DEBUG_MSG("target_path");
      BOOST_FOREACH (pt::ptree::value_type &vv, v.second) {
        if (vv.first == "point") {
          DEBUG_MSG("  point:");
          DEBUG_MSG("    key = " << vv.first);
          DEBUG_MSG("    x = " << vv.second.get<std::string>("x"));
          DEBUG_MSG("    y = " << vv.second.get<std::string>("y"));
          double x = std::stod(vv.second.get<std::string>("x"));
          double y = std::stod(vv.second.get<std::string>("y"));
          Point2D point(x, y);
          target_path_.push_back(point);
        }
      }
    }

    if (v.first == "lane_line") {
      DEBUG_MSG("lane_line");
      std::vector<Point2D> lane_line;
      BOOST_FOREACH (pt::ptree::value_type &vv, v.second) {
        if (vv.first == "point") {
          DEBUG_MSG("  point:");
          DEBUG_MSG("    key = " << vv.first);
          DEBUG_MSG("    x = " << vv.second.get<std::string>("x"));
          DEBUG_MSG("    y = " << vv.second.get<std::string>("y"));
          double x = std::stod(vv.second.get<std::string>("x"));
          double y = std::stod(vv.second.get<std::string>("y"));
          Point2D point(x, y);
          lane_line.push_back(point);
        }
      }
      lane_lines_.push_back(lane_line);
    }

    if (v.first == "side_walk") {
      DEBUG_MSG("side_walk");
      std::vector<Point2D> side_walk;
      BOOST_FOREACH (pt::ptree::value_type &vv, v.second) {
        if (vv.first == "point") {
          DEBUG_MSG("  point:");
          DEBUG_MSG("    key = " << vv.first);
          DEBUG_MSG("    x = " << vv.second.get<std::string>("x"));
          DEBUG_MSG("    y = " << vv.second.get<std::string>("y"));
          double x = std::stod(vv.second.get<std::string>("x"));
          double y = std::stod(vv.second.get<std::string>("y"));
          Point2D point(x, y);
          side_walk.push_back(point);
        }
      }
      side_walks_.push_back(side_walk);
    }
  }

  // for stop signs
  for (int i = 0; i < stop_signs_.size(); i++) {
    stop_activated_.push_back(false);
    stop_passed_.push_back(false);
  }
  stop_activated_time_.resize(stop_signs_.size());
  stop_passed_time_.resize(stop_signs_.size());

  DEBUG_MSG("**** The end of parsing Local Map ****");

  return Status::OK();
}

std::vector<Point2D>
SpeedPlanner::GetPathByPoint(Point2D point,
                             std::vector<std::vector<Point2D>> paths) {
  for (std::vector<Point2D> path : paths) {
    int closest = closestPoint(path, point.x, point.y);
    if (closest == path.size() - 1)
      closest -= 1;
    Point2D start = path[closest], end = path[closest + 1];
    double dist = distanceToLine(start, end, point);
    if (dist < kLaneWidth / 2.0)
      return path;
  }
  return {};
}

Status
SpeedPlanner::ProcessStopSigns(std::vector<Obstacle> &filtered_obstacles) {
  DEBUG_MSG("**** Process Stop Signs ****");
  // stop sign
  time_t time = std::time(NULL);
  long cur_time = long(time);
  Point2D ego_pos = ego_vehicle_->GetPosition();
  double ego_x = ego_pos.x;
  double ego_y = ego_pos.y;
  double ego_v = ego_vehicle_->GetVelocity();

  Frenet ego_path_frenet(ego_path_);

  // for computing ego sd
  int ego_idx = closestPoint(ego_path_, ego_x, ego_y);
  Point_Frenet ego_sd;
  ego_path_frenet.ToFrenet(ego_pos, ego_idx, ego_sd);
  double ego_s = ego_sd.s;
  double ego_d = ego_sd.d;
  double stop_s;

  for (int i = 0; i < stop_signs_.size(); i++) {
    if (stop_passed_[i]) {
      continue;
    }

    std::vector<Point2D> stop_sign = stop_signs_[i];
    Point2D stop_pos =
        getIntersectionPointWithCurve(stop_sign[0], stop_sign[1], ego_path_);

    // for computing stop sd
    int stop_idx = closestPoint(ego_path_, stop_pos.x, stop_pos.y);
    Point_Frenet stop_sd;
    ego_path_frenet.ToFrenet(stop_pos, stop_idx, stop_sd);
    stop_s = stop_sd.s;
    double dist = abs(stop_s - ego_s) - kEgoCarLength / 2.0;

    // construct stop sign as a obstacle
    TrajectoryPoint traj_p(stop_pos.x, stop_pos.y, 0.0, 0.0);
    Obstacle stop_obstacle(ObstacleType::STOPSIGN, {traj_p});
    filtered_obstacles.push_back(stop_obstacle);

    // reach the stop line and stop
    if (!stop_activated_[i] && dist < kArriveTolerance &&
        ego_v < kStopTolerance) {
      stop_activated_[i] = true;
      time = std::time(NULL);
      stop_activated_time_[i] = long(time);
    }

    // check if time is up
    if (stop_activated_[i]) {
      if (double(cur_time - stop_activated_time_[i]) >= stop_release_time_[i]) {
        stop_passed_[i] = true;
        time = std::time(NULL);
        stop_passed_time_[i] = long(time);
      }
    }

    break;
  }

  return Status::OK();
}

Status
SpeedPlanner::ProcessCrosswalks(std::vector<Obstacle> &filtered_obstacles) {
  Point2D ego_pos = ego_vehicle_->GetPosition();
  double ego_x = ego_pos.x;
  double ego_y = ego_pos.y;

  double distance_to_react = 10.0;

  Frenet ego_path_frenet(ego_path_);

  // for computing ego sd
  int ego_idx = closestPoint(ego_path_, ego_x, ego_y);
  Point_Frenet ego_sd;
  ego_path_frenet.ToFrenet(ego_pos, ego_idx, ego_sd);
  double ego_s = ego_sd.s;
  double ego_d = ego_sd.d;

  for (int i = 0; i < crosswalks_.size(); i++) {
    std::vector<Point2D> crosswalk = crosswalks_[i];

    Point2D stop_pos =
        getIntersectionPointWithCurve(crosswalk[0], crosswalk[1], ego_path_);

    // for computing stop sd
    int stop_idx = closestPoint(ego_path_, stop_pos.x, stop_pos.y);
    Point_Frenet stop_sd;
    ego_path_frenet.ToFrenet(stop_pos, stop_idx, stop_sd);
    double stop_s = stop_sd.s;

    if (stop_s - ego_s > distance_to_react) {
      // construct cross walk as a obstacle
      TrajectoryPoint traj_p(stop_pos.x, stop_pos.y, 0.0, 0.0);
      Obstacle crosswalk_obstacle(ObstacleType::CROSSWALK, {traj_p});
      filtered_obstacles.push_back(crosswalk_obstacle);
    }
  }
  return Status::OK();
}

Status
SpeedPlanner::ProcessTrafficLights(std::vector<Obstacle> &filtered_obstacles) {
  Point2D ego_pos = ego_vehicle_->GetPosition();
  double ego_x = ego_pos.x;
  double ego_y = ego_pos.y;

  Frenet ego_path_frenet(ego_path_);

  // for computing ego sd
  int ego_idx = closestPoint(ego_path_, ego_x, ego_y);
  Point_Frenet ego_sd;
  ego_path_frenet.ToFrenet(ego_pos, ego_idx, ego_sd);
  double ego_s = ego_sd.s;
  double ego_d = ego_sd.d;

  for (int i = 0; i < traffic_lights_.size(); i++) {
    std::vector<Point2D> tl = traffic_lights_[i];
    Point2D stop_pos = getIntersectionPointWithCurve(tl[0], tl[1], ego_path_);

    // for computing stop sd
    int stop_idx = closestPoint(ego_path_, stop_pos.x, stop_pos.y);
    Point_Frenet stop_sd;
    ego_path_frenet.ToFrenet(stop_pos, stop_idx, stop_sd);
    double stop_s = stop_sd.s;

    double dis_to_tfl = stop_s - ego_s - kEgoCarLength / 2.0;
    DEBUG_MSG("distance to tfl: " << dis_to_tfl);

    if (dis_to_tfl > 0) {
      if (tfl_status_ == RED || tfl_status_ == YELLOW) {
        // construct tflight as a obstacle
        TrajectoryPoint traj_p(stop_pos.x, stop_pos.y, 0.0, 0.0);
        Obstacle tl_obstacle(ObstacleType::TRAFFICLIGHT, {traj_p});
        filtered_obstacles.push_back(tl_obstacle);
      }
      break;
    } else {
      continue;
    }
  }

  return Status::OK();
}

Status SpeedPlanner::ProcessOtherCars(
    const std::vector<Obstacle> &perception_obstacles,
    std::vector<Obstacle> &filtered_obstacles) {
  DEBUG_MSG("[Debug] "
            << "Function: Process Other Cars.");

  Point2D ego_pos = ego_vehicle_->GetPosition();
  double ego_x = ego_pos.x;
  double ego_y = ego_pos.y;

  for (Obstacle obs : perception_obstacles) {
    if (obs.GetType() == ObstacleType::CAR) {
      DEBUG_MSG("[Debug] Type: "
                << "CAR");
      Point2D obs_pos = obs.GetPosition();
      double obs_x = obs_pos.x;
      double obs_y = obs_pos.y;
      double obs_speed = obs.GetSpeed();
      bool in_target_lane = obs.IsInTargetLane();

      bool skip = false;

      if (in_target_lane) {
        Frenet ego_path_frenet(ego_path_);

        // computing ego sd
        int ego_idx = closestPoint(ego_path_, ego_x, ego_y);
        Point_Frenet ego_sd;
        ego_path_frenet.ToFrenet(ego_pos, ego_idx, ego_sd);
        double ego_s = ego_sd.s;
        double ego_d = ego_sd.d;

        // computing obs sd
        int obs_idx = closestPoint(ego_path_, obs_x, obs_y);
        Point_Frenet obs_sd;
        ego_path_frenet.ToFrenet(obs_pos, obs_idx, obs_sd);
        double obs_s = obs_sd.s;
        double obs_d = obs_sd.d;

        DEBUG_MSG("[Debug] Target lane State: "
                  << "ego_s = " << ego_s << ", car_s = " << obs_s << ", car_d"
                  << obs_d << ", car_speed = " << obs_speed);

        // filter out other cars behind in the same lane
        // filter out other cars far in the front in the same lane
        // if (left_hand_) {
        //   skip = obs_d > kEgoCarLength;
        // } else {
        //   skip = obs_d < -kEgoCarLength;
        // }
      } else {
        // check if passed the collision point
        Frenet ego_path_frenet(ego_path_);

        // computing collision point sd
        int ego_idx = closestPoint(ego_path_, ego_x, ego_y);
        Point_Frenet ego_sd;
        ego_path_frenet.ToFrenet(ego_pos, ego_idx, ego_sd);
        double ego_s = ego_sd.s;
        double ego_d = ego_sd.d;

        // computing obs sd
        int obs_idx = closestPoint(ego_path_, obs_x, obs_y);
        Point_Frenet obs_sd;
        ego_path_frenet.ToFrenet(obs_pos, obs_idx, obs_sd);
        double obs_s = obs_sd.s;
        double obs_d = obs_sd.d;

        DEBUG_MSG("[Debug] Oncoming lane: State: "
                  << "ego_s = " << ego_s << ", car_s = " << obs_s
                  << ", car_d = " << obs_d << ", car_speed = " << obs_speed);

        // if (left_hand_) {
        //   skip = obs_d > 0;
        // } else {
        //   skip = obs_d < 0;
        // }
      }

      if (!skip)
        filtered_obstacles.push_back(obs);
    }
  }

  DEBUG_MSG("[Debug] "
            << "end");

  return Status::OK();
}

Status SpeedPlanner::ProcessPedestrians(
    const std::vector<Obstacle> &perception_obstacles,
    std::vector<Obstacle> &filtered_obstacles, double t_range, double unit_time,
    std::vector<STArea> &occupied_areas) {
  DEBUG_MSG("[Debug] "
            << "Function: Process Pedestrians on the crosswalk.");

  // Deal with pedestrian as an obstalce
  for (Obstacle obs : perception_obstacles) {
    if (obs.GetType() == ObstacleType::PEDESTRIAN) {
      DEBUG_MSG("[Debug] Type: "
                << "PEDESTRIAN");
      filtered_obstacles.push_back(obs);
    }
  }

  // Deal with pedestrian on the crosswalk
  Point2D ego_pos = ego_vehicle_->GetPosition();
  double ego_x = ego_pos.x;
  double ego_y = ego_pos.y;

  Frenet ego_path_frenet(ego_path_);

  // for computing ego sd
  int ego_idx = closestPoint(ego_path_, ego_x, ego_y);
  Point_Frenet ego_sd;
  ego_path_frenet.ToFrenet(ego_pos, ego_idx, ego_sd);
  double ego_s = ego_sd.s;
  double ego_d = ego_sd.d;

  for (int ci = 0; ci < crosswalks_.size(); ci++) {
    std::vector<Point2D> crosswalk = crosswalks_[ci];
    Point2D stop_pos =
        getIntersectionPointWithCurve(crosswalk[0], crosswalk[1], ego_path_);

    double crosswalk_span = crosswalk_span_;
    double crosswalk_shift = crosswalk_shift_;

    // for computing stop sd
    int stop_idx = closestPoint(ego_path_, stop_pos.x, stop_pos.y);
    Point_Frenet stop_sd;
    ego_path_frenet.ToFrenet(stop_pos, stop_idx, stop_sd);
    double stop_s = stop_sd.s;

    DEBUG_MSG("[Debug] ego_s = " << ego_s << ", stop_s = " << stop_s);

    if (ego_s + kEgoCarLength / 2.0 > stop_s + crosswalk_span / 2.0)
      continue;

    // get crosswalk polygon
    Point2D pl = crosswalk[0];
    Point2D pr = crosswalk[1];
    Point2D pm((pl.x + pr.x) / 2, (pl.y + pr.y) / 2);

    // vector angle
    double alpha = atan2(pr.y - pl.y, pr.x - pl.x);
    while (alpha > M_PI)
      alpha -= M_PI * 2;
    while (alpha < -M_PI)
      alpha += M_PI * 2;
    DEBUG_MSG("  alpha = " << alpha);

    // transform to local frame, pl as center
    Point2D pl_local(0, 0);
    Point2D pr_local = globalToLocal(pl, alpha, pr);
    Point2D pm_local((pl_local.x + pr_local.x) / 2,
                     (pl_local.y + pr_local.y) / 2);

    double crosswalk_thresh = 2.0;

    // ego pos local
    Point2D ego_pos_local = globalToLocal(pl, alpha, ego_pos);

    for (Obstacle obs : perception_obstacles) {
      if (obs.GetType() == ObstacleType::PEDESTRIAN) {
        Point2D ped_p = obs.GetPosition();
        DEBUG_MSG("  ped initial pos: " << ped_p.x << ", " << ped_p.y);

        /*** define walkable area ***/
        // crosswalk area
        double xlim_l = pl_local.x - crosswalk_thresh;
        double xlim_h = pr_local.x + crosswalk_thresh;
        double ylim_l = -crosswalk_span / 2.0;
        double ylim_h = +crosswalk_span / 2.0;

        // if the ego car has passed the middle line, only check the area in
        // front of the ego car
        if (ego_s + kEgoCarLength / 2.0 > stop_s) {
          xlim_l = ego_pos_local.x - kEgoCarWidth;
          xlim_h = ego_pos_local.x + kEgoCarWidth;
          ylim_l = ego_pos_local.y + kEgoCarLength / 2.0;
          ylim_h = +crosswalk_span / 2.0;
        }

        if (ped_p.DistanceTo(stop_pos) < 30) {
          std::vector<TrajectoryPoint> traj =
              obs.PredictTrajectoryOverTime(t_range, unit_time);
          double traj_angle =
              atan2(traj[1].y - traj[0].y, traj[1].x - traj[0].x);
          while (traj_angle > M_PI)
            traj_angle -= M_PI * 2;
          while (traj_angle < -M_PI)
            traj_angle += M_PI * 2;
          DEBUG_MSG("  traj_angle = " << traj_angle);
          // when pedestrian is moving away from the ego car
          // or the ego car passes over the middle line
          // if(abs(alpha-traj_angle) < M_PI/8) {
          //   xlim_h = xlim_h / 2.0;
          // }
          DEBUG_MSG("  xlim_l = " << xlim_l << ", xlim_h = " << xlim_h
                                  << ", ylim_l = " << ylim_l
                                  << ", ylim_h = " << ylim_h);
          STArea ped_st_area;
          double s_in = stop_s - (ego_s + kEgoCarLength / 2.0);
          double s_out = stop_s - (ego_s + kEgoCarLength / 2.0);
          double t_in = t_range, t_out = 0;
          for (int i = 0; i < traj.size(); i++) {
            Point2D ped_pp(traj[i].x, traj[i].y);
            if (i < 5) {
              DEBUG_MSG("  ped global: " << ped_pp.x << ", " << ped_pp.y);
            }
            Point2D ped_pp_local = globalToLocal(pl, alpha, ped_pp);
            if (i == 0) {
              DEBUG_MSG("  ped transformed: " << ped_pp_local.x << ", "
                                              << ped_pp_local.y);
            }
            if ((ped_pp_local.x > xlim_l) && (ped_pp_local.x < xlim_h)) {
              t_in = traj[i].relative_time;
              break;
            }
          }
          for (int i = traj.size() - 1; i >= 0; i--) {
            Point2D ped_pp(traj[i].x, traj[i].y);
            Point2D ped_pp_local = globalToLocal(pl, alpha, ped_pp);
            if ((ped_pp_local.x > xlim_l) && (ped_pp_local.x < xlim_h)) {
              t_out = traj[i].relative_time;
              break;
            }
          }
          DEBUG_MSG("[Debug] Pre: Cut in: " << s_in << ", " << t_in
                                            << "; Cut out: " << s_out << ", "
                                            << t_out);
          if (t_in < t_out) {
            STPoint cut_in(s_in, 0);
            STPoint cut_out(s_out, 10);

            DEBUG_MSG("[Debug] Post: Cut in: " << s_in << ", " << t_in
                                               << "; Cut out: " << s_out << ", "
                                               << t_out);

            occupied_areas.push_back(STArea(cut_in, cut_out,
                                            crosswalk_span + crosswalk_shift,
                                            crosswalk_span - crosswalk_shift));

            // TODO: construct cross walk as a obstacle
            // TrajectoryPoint traj_p1(stop_pos.x, stop_pos.y, 0.0, t_in);
            // TrajectoryPoint traj_p2(stop_pos.x, stop_pos.y, 0.0, t_out);
            // Obstacle crosswalk_obstacle(ObstacleType::CROSSWALK, {traj_p1,
            // traj_p2}); filtered_obstacles.push_back(crosswalk_obstacle);
          }
        }
      }
    }
  }

  return Status::OK();
}

Status SpeedPlanner::ProcessIntersectionArea(
    const std::vector<Obstacle> &perception_obstacles, double t_range,
    double unit_time, std::vector<STArea> &occupied_areas) {
  Point2D ego_pos = ego_vehicle_->GetPosition();
  double ego_x = ego_pos.x;
  double ego_y = ego_pos.y;

  Frenet ego_path_frenet(ego_path_);

  // for computing ego sd
  int ego_idx = closestPoint(ego_path_, ego_x, ego_y);
  Point_Frenet ego_sd;
  ego_path_frenet.ToFrenet(ego_pos, ego_idx, ego_sd);
  double ego_s = ego_sd.s;
  double ego_d = ego_sd.d;

  // start_s
  std::vector<Point2D> cross_start = crosswalks_[0];
  Point2D start_pos =
      getIntersectionPointWithCurve(cross_start[0], cross_start[1], ego_path_);
  int start_idx = closestPoint(ego_path_, start_pos.x, start_pos.y);
  Point_Frenet start_sd;
  ego_path_frenet.ToFrenet(start_pos, start_idx, start_sd);
  double start_s = start_sd.s;

  // end_s
  std::vector<Point2D> cross_end = crosswalks_[1];
  Point2D end_pos =
      getIntersectionPointWithCurve(cross_end[0], cross_end[1], ego_path_);
  int end_idx = closestPoint(ego_path_, end_pos.x, end_pos.y);
  Point_Frenet end_sd;
  ego_path_frenet.ToFrenet(end_pos, end_idx, end_sd);
  double end_s = end_sd.s;

  DEBUG_MSG("ego_s = " << ego_s << ", start_s = " << start_s
                       << ", end_s = " << end_s);

  double crosswalk_span = crosswalk_span_;

  if (ego_s + kEgoCarLength / 2.0 < start_s - crosswalk_span / 2.0)
    return Status::OK();
  if (ego_s + kEgoCarLength / 2.0 > start_s + crosswalk_span / 2.0)
    return Status::OK();

  // get intersection area
  Point2D pl = cross_start[0];
  Point2D pr = cross_start[1];

  // vector angle
  double alpha = atan2(pr.y - pl.y, pr.x - pl.x);
  while (alpha > M_PI)
    alpha -= M_PI * 2;
  while (alpha < -M_PI)
    alpha += M_PI * 2;
  DEBUG_MSG("intersection alpha = " << alpha);

  // transform to local frame, pl as center
  Point2D pl_local(0, 0);
  Point2D pr_local = globalToLocal(pl, alpha, pr);

  for (Obstacle obs : perception_obstacles) {
    if (obs.GetType() == ObstacleType::CAR) {
      Point2D intercar_p = obs.GetPosition();
      DEBUG_MSG("intersection car initial pos: " << intercar_p.x << ", "
                                                 << intercar_p.y);

      if (isOnPath(ego_path_, intercar_p, kLaneWidth / 2.0)) {
        Frenet ego_path_frenet(ego_path_);

        // computing obs sd
        int obs_idx = closestPoint(ego_path_, intercar_p.x, intercar_p.y);
        Point_Frenet obs_sd;
        ego_path_frenet.ToFrenet(intercar_p, obs_idx, obs_sd);
        double obs_s = obs_sd.s;

        // filter out other cars behind in the same lane
        // filter out other cars far in the front in the same lane
        if (ego_s > obs_s)
          continue;
      }

      /*** define drivable area ***/
      double xlim_l = pl_local.x - crosswalk_span;
      double xlim_h = pr_local.x + crosswalk_span;
      double ylim_l = crosswalk_span / 2.0;
      double ylim_h = crosswalk_span / 2.0 + 5.0 * 2;

      std::vector<TrajectoryPoint> traj =
          obs.PredictTrajectoryOverTime(t_range, unit_time);
      double traj_angle = atan2(traj[1].y - traj[0].y, traj[1].x - traj[0].x);
      while (traj_angle > M_PI)
        traj_angle -= M_PI * 2;
      while (traj_angle < -M_PI)
        traj_angle += M_PI * 2;
      DEBUG_MSG("traj_angle = " << traj_angle);
      DEBUG_MSG("xlim_l = " << xlim_l << ", xlim_h = " << xlim_h
                            << ", ylim_l = " << ylim_l
                            << ", ylim_h = " << ylim_h);

      // construct st area in st map
      STArea inter_st_area;
      double s_in = start_s - (ego_s + kEgoCarLength / 2.0);
      double s_out = end_s - (ego_s + kEgoCarLength / 2.0);
      double t_in = t_range, t_out = 0;
      for (int i = 0; i < traj.size(); i++) {
        Point2D car_pp(traj[i].x, traj[i].y);
        if (i < 5) {
          DEBUG_MSG("inter car global: " << car_pp.x << ", " << car_pp.y);
        }
        Point2D car_pp_local = globalToLocal(pl, alpha, car_pp);
        if (i == 0) {
          DEBUG_MSG("inter car transformed: " << car_pp_local.x << ", "
                                              << car_pp_local.y);
        }
        if ((car_pp_local.x > xlim_l) && (car_pp_local.x < xlim_h) &&
            (car_pp_local.y > ylim_l) && (car_pp_local.y < ylim_h)) {
          t_in = traj[i].relative_time;
          break;
        }
      }
      for (int i = traj.size() - 1; i >= 0; i--) {
        Point2D car_pp(traj[i].x, traj[i].y);
        Point2D car_pp_local = globalToLocal(pl, alpha, car_pp);
        if ((car_pp_local.x > xlim_l) && (car_pp_local.x < xlim_h) &&
            (car_pp_local.y > ylim_l) && (car_pp_local.y < ylim_h)) {
          t_out = traj[i].relative_time;
          break;
        }
      }
      DEBUG_MSG("Pre: Cut in: " << s_in << ", " << t_in
                                << "; Cut out: " << s_out << ", " << t_out);
      if (t_in < t_out) {
        STPoint cut_in(s_in, t_in);
        STPoint cut_out(s_in, t_out);
        DEBUG_MSG("Post: Cut in: " << s_in << ", " << t_in
                                   << "; Cut out: " << s_out << ", " << t_out);
        double obs_length =
            obs.GetLength() * 2 + obs.GetSpeed() * (t_out - t_in);
        occupied_areas.push_back(STArea(cut_in, cut_out, obs_length / 2.0,
                                        (s_out - s_in) + obs_length / 2.0));
      }
    }
  }

  return Status::OK();
}

Status SpeedPlanner::SetTrafficLightStatus(TrafficLightStatus status) {
  DEBUG_MSG(" Traffic Lighte Status: " << status);
  tfl_status_ = status;
  return Status::OK();
}

Status SpeedPlanner::SetSpeedLimit(double max_speed) {
  ego_vehicle_->SetSpeedLimit(max_speed);
}

Status SpeedPlanner::SetAccelerationLimits(double min_acc, double max_acc) {
  ego_vehicle_->SetThrottleMax(max_acc);
  ego_vehicle_->SetBrakeMax(-min_acc);
}

Status SpeedPlanner::SetJerkLimits(double min_jerk, double max_jerk) {
  ego_vehicle_->SetThrottleDotMax(max_jerk);
  ego_vehicle_->SetBrakeDotMax(-min_jerk);
}

Status
SpeedPlanner::SetStopSign(const std::vector<std::vector<double>> &stop_line) {
  DEBUG_MSG("Setting stop_sign ...");
  std::vector<Point2D> stop_sign;
  double release_time = release_time_;

  for (int i = 0; i < stop_line.size(); i++) {
    double x = stop_line[i][0];
    double y = stop_line[i][1];
    Point2D point(x, y);
    stop_sign.push_back(point);
  }

  stop_signs_.push_back(stop_sign);
  stop_release_time_.push_back(release_time);
  stop_activated_.push_back(false);
  stop_passed_.push_back(false);
  stop_activated_time_.push_back(0);
  stop_passed_time_.push_back(0);

  return Status::OK();
}

Status SpeedPlanner::ResetStopSign() {
  DEBUG_MSG("Resetting stop_sign ...");

  for (int i = 0; i < stop_signs_.size(); i++) {
    stop_activated_[i] = false;
    stop_passed_[i] = false;
    stop_activated_time_[i] = 0;
    stop_passed_time_[i] = 0;
  }

  return Status::OK();
}

Status
SpeedPlanner::SetCrossWalk(const std::vector<std::vector<double>> &cross_walk) {
  DEBUG_MSG("Setting cross_walk ...");
  Point2D p1(cross_walk[0][0], cross_walk[0][1]);
  Point2D p2(cross_walk[1][0], cross_walk[1][1]);
  std::vector<Point2D> crosswalk = {p1, p2};
  double crosswalk_span = 2.0;
  crosswalks_.push_back(crosswalk);
  crosswalk_spans_.push_back(crosswalk_span);
  return Status::OK();
}

double SpeedPlanner::NextVelocityCommand() {
  if (cmd_vels_.empty())
    return -1.0;
  double next_velocity_command = cmd_vels_[0];
  cmd_vels_.erase(cmd_vels_.begin());
  return next_velocity_command;
}

// Status SpeedPlanner::SetActuatorsLatency(double latency){
//    ego_vehicle_->SetActuatorsLatency(latency);
// }

} // namespace planning
