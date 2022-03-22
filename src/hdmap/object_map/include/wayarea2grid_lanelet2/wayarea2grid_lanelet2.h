/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 */
#ifndef WAYAREA2GRID_LANELET2_WAYAREA2GRID_LANELET2_H
#define WAYAREA2GRID_LANELET2_WAYAREA2GRID_LANELET2_H

#include <iostream>
#include <string>
#include <vector>

// Headers from ROS packages
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Headers from Autoware
#include <autoware_lanelet2_msgs/MapBin.h>
#include <grid_map_msgs/GridMap.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <object_map/object_map_utils.hpp>

// Headers from opencv
#include <opencv2/highgui/highgui.hpp>

namespace object_map
{
constexpr int occupancy_road = 128;
constexpr int occupancy_no_road = 255;

class WayareaToGridLanelet2
{
public:
  WayareaToGridLanelet2();

  void Run();

private:
  // handle
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  ros::Publisher publisher_grid_map_;
  ros::Publisher publisher_occupancy_;

  grid_map::GridMap gridmap_;

  bool loaded_lanelet_map_ = false;
  lanelet::LaneletMapPtr lanelet_map_;

  std::string sensor_frame_;
  std::string grid_frame_;

  const std::string grid_layer_name_ = "wayarea";

  double grid_resolution_;
  double grid_length_x_;
  double grid_length_y_;
  double grid_position_x_;
  double grid_position_y_;
  double grid_position_z_;

  tf::TransformListener tf_listener_;

  const int grid_min_value_ = 0;
  const int grid_max_value_ = 255;

  std::vector<std::vector<geometry_msgs::Point>> area_points_;

  /*!
   * Initializes ROS Publisher, Subscribers and sets the configuration parameters
   */
  void InitializeROSIo();
  void laneletBinMapCallback(const autoware_lanelet2_msgs::MapBin& msg);
  void initAreaPointsFromLaneletMap();
};

}  // namespace object_map

#endif  // WAYAREA2GRID_LANELET2_WAYAREA2GRID_LANELET2_H
