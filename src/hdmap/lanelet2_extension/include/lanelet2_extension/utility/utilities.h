/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
 * Authors: Kenji Miyake, Ryohsuke Mitsudome
 */

#ifndef LANELET2_EXTENSION_UTILITY_UTILITIES_H
#define LANELET2_EXTENSION_UTILITY_UTILITIES_H

#include <geometry_msgs/Point.h>

#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <autoware_msgs/LaneArray.h>

#include <map>

namespace lanelet
{
namespace utils
{
/**
 * [matchWaypointAndLanelet Matches waypoints and lanelets]
 * @param lanelet_map          [pointer to lanelet2 map]
 * @param routing_graph        [roughting graph of the map]
 * @param lane_array           [lane array containing waypoints]
 * @param waypointid2laneletid [object with key:"gid(gobal_id) of waypoints"
 * value:"lanelet id"]
 */
void matchWaypointAndLanelet(const lanelet::LaneletMapPtr lanelet_map,
                             const lanelet::routing::RoutingGraphPtr routing_graph,
                             const autoware_msgs::LaneArray& lane_array,
                             std::map<int, lanelet::Id>* waypointid2laneletid);

/**
 * @brief  Apply a patch for centerline because the original implementation
 * doesn't have enough quality
 */
void overwriteLaneletsCenterline(lanelet::LaneletMapPtr lanelet_map, const bool force_overite = false);

}  // namespace utils
}  // namespace lanelet

#endif  // LANELET2_EXTENSION_UTILITY_UTILITIES_H
