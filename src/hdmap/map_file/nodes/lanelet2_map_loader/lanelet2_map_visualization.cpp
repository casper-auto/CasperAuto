/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/LaneletMap.h>
#include <autoware_lanelet2_msgs/MapBin.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>

#include <vector>

static bool g_viz_lanelets_centerline = true;
static ros::Publisher g_map_pub;

void insertMarkerArray(visualization_msgs::MarkerArray* a1, const visualization_msgs::MarkerArray& a2)
{
  a1->markers.insert(a1->markers.end(), a2.markers.begin(), a2.markers.end());
}

void setColor(std_msgs::ColorRGBA* cl, double r, double g, double b, double a)
{
  cl->r = r;
  cl->g = g;
  cl->b = b;
  cl->a = a;
}

void binMapCallback(autoware_lanelet2_msgs::MapBin msg)
{
  lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(msg, viz_lanelet_map);
  ROS_INFO("Map loaded");

  // get lanelets etc to visualize
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  lanelet::ConstLanelets crosswalk_lanelets = lanelet::utils::query::crosswalkLanelets(all_lanelets);

  std::vector<lanelet::ConstLineString3d> tl_stop_lines = lanelet::utils::query::getTrafficLightStopLines(road_lanelets);
  std::vector<lanelet::ConstLineString3d> ss_stop_lines = lanelet::utils::query::getStopSignStopLines(road_lanelets);
  std::vector<lanelet::TrafficLightConstPtr> tl_reg_elems = lanelet::utils::query::trafficLights(all_lanelets);
  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems =
      lanelet::utils::query::autowareTrafficLights(all_lanelets);

  std_msgs::ColorRGBA cl_road, cl_cross, cl_ll_borders, cl_tl_stoplines, cl_ss_stoplines, cl_trafficlights;
  setColor(&cl_road, 0.2, 0.7, 0.7, 0.3);
  setColor(&cl_cross, 0.2, 0.7, 0.2, 0.3);
  setColor(&cl_ll_borders, 1.0, 1.0, 1.0, 1.0);
  setColor(&cl_tl_stoplines, 1.0, 0.5, 0.0, 0.5);
  setColor(&cl_ss_stoplines, 1.0, 0.0, 0.0, 0.5);
  setColor(&cl_trafficlights, 0.7, 0.7, 0.7, 0.8);

  visualization_msgs::MarkerArray map_marker_array;

  insertMarkerArray(&map_marker_array, lanelet::visualization::laneletsBoundaryAsMarkerArray(
    road_lanelets, cl_ll_borders, g_viz_lanelets_centerline));
  insertMarkerArray(&map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
    "road_lanelets", road_lanelets, cl_road));
  insertMarkerArray(&map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
    "crosswalk_lanelets", crosswalk_lanelets, cl_cross));
  insertMarkerArray(&map_marker_array, lanelet::visualization::laneletDirectionAsMarkerArray(
    road_lanelets));
  insertMarkerArray(&map_marker_array, lanelet::visualization::lineStringsAsMarkerArray(
    tl_stop_lines, "traffic_light_stop_lines", cl_tl_stoplines, 0.5));
  insertMarkerArray(&map_marker_array, lanelet::visualization::lineStringsAsMarkerArray(
    ss_stop_lines, "stop_sign_stop_lines", cl_ss_stoplines, 0.5));
  insertMarkerArray(&map_marker_array, lanelet::visualization::autowareTrafficLightsAsMarkerArray(
    aw_tl_reg_elems, cl_trafficlights));

  ROS_INFO("Visualizing lanelet2 map with %lu lanelets, %lu stop lines, and %lu traffic lights",
    all_lanelets.size(), tl_stop_lines.size() + ss_stop_lines.size(), aw_tl_reg_elems.size());

  g_map_pub.publish(map_marker_array);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lanelet_map_visualizer");
  ros::NodeHandle rosnode;
  ros::Subscriber bin_map_sub;

  bin_map_sub = rosnode.subscribe("/lanelet_map_bin", 1, binMapCallback);
  g_map_pub = rosnode.advertise<visualization_msgs::MarkerArray>("lanelet2_map_viz", 1, true);

  ros::spin();

  return 0;
}
