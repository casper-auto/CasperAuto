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
 ********************/

#include <wayarea2grid_lanelet2/wayarea2grid_lanelet2.h>

#include <string>
#include <vector>

namespace object_map
{
WayareaToGridLanelet2::WayareaToGridLanelet2() : private_node_handle_("~")
{
  InitializeROSIo();
  initAreaPointsFromLaneletMap();
}

void WayareaToGridLanelet2::initAreaPointsFromLaneletMap()
{
  ros::Subscriber sub_lanelet_bin_map =
      node_handle_.subscribe("lanelet_map_bin", 1, &WayareaToGridLanelet2::laneletBinMapCallback, this);

  while (ros::ok() && !loaded_lanelet_map_)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  // use all lanelets in map of subtype road to give way area
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);

  // convert lanelets to polygons and put into area_points array
  for (const auto& ll : road_lanelets)
  {
    std::vector<geometry_msgs::Polygon> triangles;
    lanelet::visualization::lanelet2Triangle(ll, &triangles);

    for (const auto& triangle : triangles)
    {
      std::vector<geometry_msgs::Point> poly_pts;
      for (const auto& p : triangle.points)
      {
        // convert from Point32 to Point
        geometry_msgs::Point gp;
        gp.x = p.x;
        gp.y = p.y;
        gp.z = p.z;
        poly_pts.push_back(gp);
      }
      area_points_.push_back(poly_pts);
    }
  }
}

void WayareaToGridLanelet2::InitializeROSIo()
{
  private_node_handle_.param<std::string>("sensor_frame", sensor_frame_, "velodyne");
  private_node_handle_.param<std::string>("grid_frame_", grid_frame_, "map");
  private_node_handle_.param<double>("grid_resolution", grid_resolution_, 0.2);
  private_node_handle_.param<double>("grid_length_x", grid_length_x_, 80);
  private_node_handle_.param<double>("grid_length_y", grid_length_y_, 30);
  private_node_handle_.param<double>("grid_position_x", grid_position_x_, 20);
  private_node_handle_.param<double>("grid_position_y", grid_position_y_, 0);
  private_node_handle_.param<double>("grid_position_z", grid_position_z_, -2.f);

  publisher_grid_map_ = node_handle_.advertise<grid_map_msgs::GridMap>("grid_map_wayarea", 1, true);
  publisher_occupancy_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("occupancy_wayarea", 1, true);
}

void WayareaToGridLanelet2::laneletBinMapCallback(const autoware_lanelet2_msgs::MapBin& msg)
{
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_);
  loaded_lanelet_map_ = true;
}

void WayareaToGridLanelet2::Run()
{
  ros::Rate loop_rate(10);

  gridmap_.add(grid_layer_name_);
  gridmap_.setFrameId(sensor_frame_);
  gridmap_.setGeometry(grid_map::Length(grid_length_x_, grid_length_y_), grid_resolution_,
                       grid_map::Position(grid_position_x_, grid_position_y_));

  while (ros::ok())
  {
    if (!area_points_.empty())
    {
      FillPolygonAreas(gridmap_, area_points_, grid_layer_name_, occupancy_no_road, occupancy_road, grid_min_value_,
                       grid_max_value_, sensor_frame_, grid_frame_, tf_listener_);
      PublishGridMap(gridmap_, publisher_grid_map_);
      PublishOccupancyGrid(gridmap_, publisher_occupancy_, grid_layer_name_, grid_min_value_, grid_max_value_,
                           grid_position_z_);
    }

    loop_rate.sleep();
  }
}

}  // namespace object_map
