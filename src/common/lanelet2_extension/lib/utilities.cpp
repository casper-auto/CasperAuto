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
 *
 */

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <lanelet2_extension/utility/utilities.h>
#include <ros/ros.h>

#include <algorithm>
#include <map>
#include <utility>
#include <vector>

namespace lanelet
{
namespace utils
{
namespace
{
bool exists(const std::vector<int>& array, const int element)
{
  return std::find(array.begin(), array.end(), element) != array.end();
}

/**
 * [removeImpossibleCandidates eliminates the impossible lanelet id candidates
 * according to lanelet routing graph information ]
 * @method removeImpossibleCandidates
 * @param  waypoints [list of waypoints]
 * @param  wp_candidate_lanelets  list of lanelet id candidates for each
 * waypoint
 */
void removeImpossibleCandidates(const lanelet::LaneletMapPtr lanelet_map,
                                const lanelet::routing::RoutingGraphPtr routing_graph,
                                const std::vector<autoware_msgs::Waypoint>& waypoints,
                                std::map<int, std::vector<int> >* wp_candidate_lanelets, const bool reverse)
{
  if (!lanelet_map)
  {
    ROS_ERROR_STREAM("No lanelet map is set!");
    return;
  }

  int prev_wp_gid;
  bool first_wp = true;
  for (const auto& wp : waypoints)
  {
    if (first_wp)
    {
      prev_wp_gid = wp.gid;
      first_wp = false;
      continue;
    }

    auto candidate_ids_ptr = &wp_candidate_lanelets->at(wp.gid);
    const auto prev_wp_candidate_ids_ptr = &wp_candidate_lanelets->at(prev_wp_gid);

    // do not eliminate if previous waypoint do not have any candidate
    if (prev_wp_candidate_ids_ptr->empty())
      continue;

    std::vector<int> removing_ids;
    for (const auto candidate_id : *candidate_ids_ptr)
    {
      // do not eliminate if the belonging lane exists in candidates of previous
      // waypoint
      if (exists(*prev_wp_candidate_ids_ptr, candidate_id))
        continue;

      auto lanelet = lanelet_map->laneletLayer.get(candidate_id);
      // get available previous lanelet from routing graph
      lanelet::ConstLanelets previous_lanelets;
      if (reverse)
      {
        previous_lanelets = routing_graph->following(lanelet);
      }
      else
      {
        previous_lanelets = routing_graph->previous(lanelet);
      }

      // connection is impossible if none of predecessor lanelets match with
      // lanelet candidates from previous waypoint
      bool connection_possible = false;
      for (const auto& previous_lanelet : previous_lanelets)
      {
        if (exists(*prev_wp_candidate_ids_ptr, previous_lanelet.id()))
        {
          connection_possible = true;
          break;
        }
      }
      if (!connection_possible)
      {
        removing_ids.push_back(candidate_id);
      }
    }

    // declare function for remove_if separately, because roslint is not supporting lambda functions very well.
    auto remove_existing_func = [removing_ids](int id) { return exists(removing_ids, id); };

    auto result = std::remove_if(candidate_ids_ptr->begin(), candidate_ids_ptr->end(), remove_existing_func);

    candidate_ids_ptr->erase(result, candidate_ids_ptr->end());
    prev_wp_gid = wp.gid;
  }
}

/**
 * [getContactingLanelets retrieves id of lanelets which has distance 0m to
 * search_point]
 * @param  lanelet_map   [pointer to lanelet]
 * @param  trafficRules  [traffic rules to ignore lanelets that are not
 * traversible]
 * @param  search_point  [2D point used for searching]
 * @param  search_n      [initial guess used for findNearest function (default
 * 5)]
 * @param  contacting_lanelet_ids [array of lanelet ids that is contacting with
 * search_point]
 */
void getContactingLanelets(const lanelet::LaneletMapPtr lanelet_map,
                           const lanelet::traffic_rules::TrafficRulesPtr traffic_rules,
                           const lanelet::BasicPoint2d search_point, const int search_n,
                           std::vector<int>* contacting_lanelet_ids)
{
  if (!lanelet_map)
  {
    ROS_ERROR_STREAM("No lanelet map is set!");
    return;
  }

  if (contacting_lanelet_ids == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << " contacting_lanelet_ids is null pointer!");
    return;
  }

  int n = search_n;
  double max_distance = 0.0;
  const int increment = 3;
  const double epsilon = 1e-6;
  std::vector<std::pair<double, lanelet::Lanelet> > actuallyNearestLanelets;

  // keep searching nearest lanelet as long as all retrieved lanelet has
  // distance == 0m.
  while (max_distance < epsilon)
  {
    actuallyNearestLanelets = lanelet::geometry::findNearest(lanelet_map->laneletLayer, search_point, n);
    max_distance = 0.0;
    for (auto const& item : actuallyNearestLanelets)
    {
      if (item.first > max_distance)
      {
        max_distance = item.first;
      }
    }

    // exit loop if all lanelets in the map intersects with the waypoint,
    // e.g. when there is only one lanelet in the map.
    if (actuallyNearestLanelets.size() < n)
    {
      break;
    }

    n += increment;
  }

  contacting_lanelet_ids->reserve(n - increment);
  for (auto const& item : actuallyNearestLanelets)
  {
    if (item.first < epsilon && traffic_rules->canPass(item.second))
    {
      contacting_lanelet_ids->push_back(item.second.id());
    }
  }
}

std::vector<double> calculateSegmentDistances(const lanelet::ConstLineString3d& line_string)
{
  std::vector<double> segment_distances;
  segment_distances.reserve(line_string.size() - 1);

  for (size_t i = 1; i < line_string.size(); ++i)
  {
    const auto distance = lanelet::geometry::distance2d(line_string[i], line_string[i - 1]);
    segment_distances.push_back(distance);
  }

  return segment_distances;
}

std::vector<double> calculateAccumulatedLengths(const lanelet::ConstLineString3d& line_string)
{
  const auto segment_distances = calculateSegmentDistances(line_string);

  std::vector<double> accumulated_lengths{ 0 };
  accumulated_lengths.reserve(segment_distances.size() + 1);
  std::partial_sum(std::begin(segment_distances), std::end(segment_distances), std::back_inserter(accumulated_lengths));

  return accumulated_lengths;
}

std::pair<size_t, size_t> findNearestIndexPair(const std::vector<double>& accumulated_lengths,
                                               const double target_length)
{
  // List size
  const auto N = accumulated_lengths.size();

  // Front
  if (target_length < accumulated_lengths.at(1))
  {
    return std::make_pair(0, 1);
  }

  // Back
  if (target_length > accumulated_lengths.at(N - 2))
  {
    return std::make_pair(N - 2, N - 1);
  }

  // Middle
  for (auto i = 1; i < N; ++i)
  {
    if (accumulated_lengths.at(i - 1) <= target_length && target_length <= accumulated_lengths.at(i))
    {
      return std::make_pair(i - 1, i);
    }
  }

  // Throw an exception because this never happens
  throw std::runtime_error("No nearest point found.");
}

std::vector<lanelet::BasicPoint3d> resamplePoints(const lanelet::ConstLineString3d& line_string, const int num_segments)
{
  // Calculate length
  const auto line_length = lanelet::geometry::length(line_string);

  // Calculate accumulated lengths
  const auto accumulated_lengths = calculateAccumulatedLengths(line_string);

  // Create each segment
  std::vector<lanelet::BasicPoint3d> resampled_points;
  for (auto i = 0; i <= num_segments; ++i)
  {
    // Find two nearest points
    const auto target_length = (static_cast<double>(i) / num_segments) * line_length;
    const auto index_pair = findNearestIndexPair(accumulated_lengths, target_length);

    // Apply linear interpolation
    const lanelet::BasicPoint3d back_point = line_string[index_pair.first];
    const lanelet::BasicPoint3d front_point = line_string[index_pair.second];
    const auto direction_vector = (front_point - back_point);

    const auto back_length = accumulated_lengths.at(index_pair.first);
    const auto front_length = accumulated_lengths.at(index_pair.second);
    const auto segment_length = front_length - back_length;
    const auto target_point = back_point + (direction_vector * (target_length - back_length) / segment_length);

    // Add to list
    resampled_points.push_back(target_point);
  }

  return resampled_points;
}

lanelet::LineString3d generateFineCenterline(const lanelet::ConstLanelet& lanelet_obj)
{
  // Parameter
  constexpr double point_interval = 1.0;  // [m]

  // Get length of longer border
  const double left_length = lanelet::geometry::length(lanelet_obj.leftBound());
  const double right_length = lanelet::geometry::length(lanelet_obj.rightBound());
  const double longer_distance = (left_length > right_length) ? left_length : right_length;
  const int num_segments = std::max(static_cast<int>(ceil(longer_distance / point_interval)), 1);

  // Resample points
  const auto left_points = resamplePoints(lanelet_obj.leftBound(), num_segments);
  const auto right_points = resamplePoints(lanelet_obj.rightBound(), num_segments);

  // Create centerline
  lanelet::LineString3d centerline(lanelet::utils::getId());
  for (int i = 0; i < num_segments + 1; i++)
  {
    // Add ID for the average point of left and right
    const auto center_basic_point = (right_points.at(i) + left_points.at(i)) / 2;
    const lanelet::Point3d center_point(lanelet::utils::getId(), center_basic_point.x(), center_basic_point.y(),
                                        center_basic_point.z());
    centerline.push_back(center_point);
  }
  return centerline;
}

}  // namespace

void matchWaypointAndLanelet(const lanelet::LaneletMapPtr lanelet_map,
                             const lanelet::routing::RoutingGraphPtr routing_graph,
                             const autoware_msgs::LaneArray& lane_array,
                             std::map<int, lanelet::Id>* waypointid2laneletid)
{
  if (!lanelet_map)
  {
    ROS_ERROR_STREAM("No lanelet map is set!");
    return;
  }

  if (waypointid2laneletid == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": waypointid2laneletid null pointer!");
    return;
  }

  // map of waypoint_id to lanelet_id
  // first item = gid of waypoint
  // second item = lanelet_ids
  std::map<int, std::vector<int> > wp_candidate_lanelet_ids;

  lanelet::traffic_rules::TrafficRulesPtr traffic_rules =
      lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);

  // get possible candidates of lanelets for each waypoint
  // "candidate lanelets" means lanelets that have 0 distance with waypoint.
  // multiple candidates could appear at intersections.
  for (auto& lane : lane_array.lanes)
  {
    for (auto& wp : lane.waypoints)
    {
      std::vector<int> contacting_lanelet_ids;
      lanelet::BasicPoint2d search_point(wp.pose.pose.position.x, wp.pose.pose.position.y);
      getContactingLanelets(lanelet_map, traffic_rules, search_point, 5, &contacting_lanelet_ids);
      wp_candidate_lanelet_ids[wp.gid] = contacting_lanelet_ids;
    }
  }

  // eliminate impossible candidates using routing graph. (forward direction)
  for (const auto& lane : lane_array.lanes)
  {
    removeImpossibleCandidates(lanelet_map, routing_graph, lane.waypoints, &wp_candidate_lanelet_ids, false);
  }

  // eliminate impossible candidates using routing graph. (reverse direction)
  for (const auto& lane : lane_array.lanes)
  {
    auto reverse_waypoints = lane.waypoints;
    std::reverse(reverse_waypoints.begin(), reverse_waypoints.end());
    removeImpossibleCandidates(lanelet_map, routing_graph, reverse_waypoints, &wp_candidate_lanelet_ids, true);
  }

  for (auto candidate : wp_candidate_lanelet_ids)
  {
    if (candidate.second.empty())
    {
      ROS_WARN_STREAM("No lanelet was matched for waypoint with gid: " << candidate.first);
      continue;
    }
    if (candidate.second.size() >= 2)
    {
      ROS_WARN("ambiguous waypoint. Randomly choosing from candidates");
    }
    (*waypointid2laneletid)[candidate.first] = candidate.second.front();
  }
}

void overwriteLaneletsCenterline(lanelet::LaneletMapPtr lanelet_map, const bool force_overwrite)
{
  for (auto& lanelet_obj : lanelet_map->laneletLayer)
  {
    if (force_overwrite || !lanelet_obj.hasCustomCenterline())
    {
      const auto fine_center_line = generateFineCenterline(lanelet_obj);
      lanelet_obj.setCenterline(fine_center_line);
    }
  }
}

}  // namespace utils
}  // namespace lanelet
