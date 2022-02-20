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
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 */

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>

#include <set>
#include <string>
#include <vector>

namespace lanelet
{
namespace utils
{
// returns all lanelets in laneletLayer - don't know how to convert
// PrimitveLayer<Lanelets> -> std::vector<Lanelets>
lanelet::ConstLanelets query::laneletLayer(const lanelet::LaneletMapPtr ll_map)
{
  lanelet::ConstLanelets lanelets;
  if (!ll_map)
  {
    ROS_WARN("No map received!");
    return lanelets;
  }

  for (auto li = ll_map->laneletLayer.begin(); li != ll_map->laneletLayer.end(); li++)
  {
    lanelets.push_back(*li);
  }

  return lanelets;
}

lanelet::ConstLanelets query::subtypeLanelets(const lanelet::ConstLanelets lls, const char subtype[])
{
  lanelet::ConstLanelets subtype_lanelets;

  for (auto li = lls.begin(); li != lls.end(); li++)
  {
    lanelet::ConstLanelet ll = *li;

    if (ll.hasAttribute(lanelet::AttributeName::Subtype))
    {
      lanelet::Attribute attr = ll.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() == subtype)
      {
        subtype_lanelets.push_back(ll);
      }
    }
  }

  return subtype_lanelets;
}

lanelet::ConstLanelets query::crosswalkLanelets(const lanelet::ConstLanelets lls)
{
  return (query::subtypeLanelets(lls, lanelet::AttributeValueString::Crosswalk));
}

lanelet::ConstLanelets query::roadLanelets(const lanelet::ConstLanelets lls)
{
  return (query::subtypeLanelets(lls, lanelet::AttributeValueString::Road));
}

std::vector<lanelet::TrafficLightConstPtr> query::trafficLights(const lanelet::ConstLanelets lanelets)
{
  std::vector<lanelet::TrafficLightConstPtr> tl_reg_elems;

  for (auto i = lanelets.begin(); i != lanelets.end(); i++)
  {
    lanelet::ConstLanelet ll = *i;
    std::vector<lanelet::TrafficLightConstPtr> ll_tl_re = ll.regulatoryElementsAs<lanelet::TrafficLight>();

    // insert unique tl into array
    for (auto tli = ll_tl_re.begin(); tli != ll_tl_re.end(); tli++)
    {
      lanelet::TrafficLightConstPtr tl_ptr = *tli;
      lanelet::Id id = tl_ptr->id();
      bool unique_id = true;
      for (auto ii = tl_reg_elems.begin(); ii != tl_reg_elems.end(); ii++)
      {
        if (id == (*ii)->id())
        {
          unique_id = false;
          break;
        }
      }
      if (unique_id)
      {
        tl_reg_elems.push_back(tl_ptr);
      }
    }
  }
  return tl_reg_elems;
}

std::vector<lanelet::AutowareTrafficLightConstPtr> query::autowareTrafficLights(const lanelet::ConstLanelets lanelets)
{
  std::vector<lanelet::AutowareTrafficLightConstPtr> tl_reg_elems;

  for (auto i = lanelets.begin(); i != lanelets.end(); i++)
  {
    lanelet::ConstLanelet ll = *i;
    std::vector<lanelet::AutowareTrafficLightConstPtr> ll_tl_re =
        ll.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>();

    // insert unique tl into array
    for (auto tli = ll_tl_re.begin(); tli != ll_tl_re.end(); tli++)
    {
      lanelet::AutowareTrafficLightConstPtr tl_ptr = *tli;
      lanelet::Id id = tl_ptr->id();
      bool unique_id = true;

      for (auto ii = tl_reg_elems.begin(); ii != tl_reg_elems.end(); ii++)
      {
        if (id == (*ii)->id())
        {
          unique_id = false;
          break;
        }
      }

      if (unique_id)
        tl_reg_elems.push_back(tl_ptr);
    }
  }
  return tl_reg_elems;
}

// return all stop lines and ref lines from a given set of lanelets
std::vector<lanelet::ConstLineString3d> query::getTrafficLightStopLines(const lanelet::ConstLanelets lanelets)
{
  std::vector<lanelet::ConstLineString3d> stoplines;

  for (auto lli = lanelets.begin(); lli != lanelets.end(); lli++)
  {
    std::vector<lanelet::ConstLineString3d> ll_stoplines;
    ll_stoplines = query::getTrafficLightStopLines(*lli);
    stoplines.insert(stoplines.end(), ll_stoplines.begin(), ll_stoplines.end());
  }

  return stoplines;
}

// return all stop and ref lines from a given lanelet
std::vector<lanelet::ConstLineString3d> query::getTrafficLightStopLines(const lanelet::ConstLanelet ll)
{
  std::vector<lanelet::ConstLineString3d> stoplines;

  // find stop lines referenced by traffic lights
  std::vector<std::shared_ptr<const lanelet::TrafficLight> > traffic_light_reg_elems =
      ll.regulatoryElementsAs<const lanelet::TrafficLight>();

  // lanelet has a traffic light elem element
  for (const auto reg_elem : traffic_light_reg_elems)
  {
    lanelet::Optional<lanelet::ConstLineString3d> traffic_light_stopline_opt = reg_elem->stopLine();
    if (!!traffic_light_stopline_opt)
    {
      stoplines.push_back(traffic_light_stopline_opt.get());
    }
  }

  return stoplines;
}

std::vector<lanelet::ConstLineString3d> query::getStopSignStopLines(const lanelet::ConstLanelets lanelets,
                                                                    const std::string& stop_sign_id)
{
  std::vector<lanelet::ConstLineString3d> all_stoplines;
  std::vector<lanelet::ConstLineString3d> traffic_sign_stoplines;
  std::vector<lanelet::ConstLineString3d> right_of_way_stoplines;
  std::vector<lanelet::ConstLineString3d> all_way_stop_stoplines;

  traffic_sign_stoplines = getTrafficSignStopLines(lanelets, stop_sign_id);
  right_of_way_stoplines = getRightOfWayStopLines(lanelets);
  all_way_stop_stoplines = getAllWayStopStopLines(lanelets);

  all_stoplines.reserve(traffic_sign_stoplines.size() + right_of_way_stoplines.size() + all_way_stop_stoplines.size());
  all_stoplines.insert(all_stoplines.end(), traffic_sign_stoplines.begin(), traffic_sign_stoplines.end());
  all_stoplines.insert(all_stoplines.end(), right_of_way_stoplines.begin(), right_of_way_stoplines.end());
  all_stoplines.insert(all_stoplines.end(), all_way_stop_stoplines.begin(), all_way_stop_stoplines.end());

  return all_stoplines;
}

std::vector<lanelet::ConstLineString3d> query::getTrafficSignStopLines(const lanelet::ConstLanelets lanelets,
                                                                       const std::string& stop_sign_id)
{
  std::vector<lanelet::ConstLineString3d> stoplines;
  std::set<lanelet::Id> checklist;

  for (const auto& ll : lanelets)
  {
    // find stop lines referenced by traffic signs
    std::vector<std::shared_ptr<const lanelet::TrafficSign> > traffic_sign_reg_elems =
        ll.regulatoryElementsAs<const lanelet::TrafficSign>();

    if (traffic_sign_reg_elems.size() > 0)
    {
      // lanelet has a traffic sign reg elem - can have multiple ref lines (but
      // stop sign should have 1
      for (const auto& ts : traffic_sign_reg_elems)
      {
        // skip if traffic sign is not stop sign
        if (ts->type() != stop_sign_id)
        {
          continue;
        }

        lanelet::ConstLineStrings3d traffic_sign_stoplines = ts->refLines();

        // only add new items
        if (traffic_sign_stoplines.size() > 0)
        {
          auto id = traffic_sign_stoplines.front().id();
          if (checklist.find(id) == checklist.end())
          {
            checklist.insert(id);
            stoplines.push_back(traffic_sign_stoplines.front());
          }
        }
      }
    }
  }
  return stoplines;
}

std::vector<lanelet::ConstLineString3d> query::getRightOfWayStopLines(const lanelet::ConstLanelets lanelets)
{
  std::vector<lanelet::ConstLineString3d> stoplines;

  for (const auto& ll : lanelets)
  {
    // find stop lines referenced by RightOfWay reg. elems.
    std::vector<std::shared_ptr<const lanelet::RightOfWay> > right_of_way_reg_elems =
        ll.regulatoryElementsAs<const lanelet::RightOfWay>();

    for (const auto reg_elem : right_of_way_reg_elems)
    {
      if (reg_elem->getManeuver(ll) == lanelet::ManeuverType::Yield)
      {
        // lanelet has a yield reg. elem.
        lanelet::Optional<lanelet::ConstLineString3d> row_stopline_opt = reg_elem->stopLine();
        if (!!row_stopline_opt)
        {
          stoplines.push_back(row_stopline_opt.get());
        }
      }
    }
  }
  return stoplines;
}

std::vector<lanelet::ConstLineString3d> query::getAllWayStopStopLines(const lanelet::ConstLanelets lanelets)
{
  std::vector<lanelet::ConstLineString3d> stoplines;

  for (const auto& ll : lanelets)
  {
    // Get every AllWayStop reg. elem. that this lanelet references.
    std::vector<std::shared_ptr<const lanelet::AllWayStop>> all_way_stop_reg_elems =
        ll.regulatoryElementsAs<const lanelet::AllWayStop>();

    for (const auto reg_elem : all_way_stop_reg_elems)
    {
      // Only get the stopline for this lanelet
      lanelet::Optional<lanelet::ConstLineString3d> stopline = reg_elem->getStopLine(ll);
      if (!!stopline)
      {
        stoplines.push_back(stopline.get());
      }
    }
  }
  return stoplines;
}

}  // namespace utils
}  // namespace lanelet
