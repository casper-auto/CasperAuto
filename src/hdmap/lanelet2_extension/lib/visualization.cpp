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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>

#include <string>
#include <vector>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>

#include <amathutils_lib/amathutils.hpp>

namespace
{
void adjacentPoints(const int i, const int N, const geometry_msgs::Polygon poly, geometry_msgs::Point32* p0,
                    geometry_msgs::Point32* p1, geometry_msgs::Point32* p2)
{
  if (p0 == nullptr || p1 == nullptr || p2 == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": either p0, p1, or p2 is null pointer!");
    return;
  }

  *p1 = poly.points[i];
  if (i == 0)
    *p0 = poly.points[N - 1];
  else
    *p0 = poly.points[i - 1];

  if (i < N - 1)
    *p2 = poly.points[i + 1];
  else
    *p2 = poly.points[0];
}

double hypot(const geometry_msgs::Point32& p0, const geometry_msgs::Point32& p1)
{
  return (sqrt(pow((p1.x - p0.x), 2.0) + pow((p1.y - p0.y), 2.0)));
}

bool isAttributeValue(const lanelet::ConstPoint3d p, const std::string attr_str, const std::string value_str)
{
  lanelet::Attribute attr = p.attribute(attr_str);
  if (attr.value().compare(value_str) == 0)
    return true;
  return false;
}

bool isLaneletAttributeValue(const lanelet::ConstLanelet ll, const std::string attr_str, const std::string value_str)
{
  lanelet::Attribute attr = ll.attribute(attr_str);
  if (attr.value().compare(value_str) == 0)
    return true;
  return false;
}

void lightAsMarker(lanelet::ConstPoint3d p, visualization_msgs::Marker* marker, const std::string ns)
{
  if (marker == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": marker is null pointer!");
    return;
  }

  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time();
  marker->ns = ns;
  marker->id = p.id();
  marker->type = visualization_msgs::Marker::SPHERE;
  marker->pose.position.x = p.x();
  marker->pose.position.y = p.y();
  marker->pose.position.z = p.z();

  float s = 0.3;

  marker->scale.x = s;
  marker->scale.y = s;
  marker->scale.z = s;

  marker->color.r = 0.0f;
  marker->color.g = 0.0f;
  marker->color.b = 0.0f;
  marker->color.a = 1.0f;

  if (isAttributeValue(p, "color", "red"))
    marker->color.r = 1.0f;
  else if (isAttributeValue(p, "color", "green"))
    marker->color.g = 1.0f;
  else if (isAttributeValue(p, "color", "yellow"))
  {
    marker->color.r = 1.0f;
    marker->color.g = 1.0f;
  }
  else
  {
    marker->color.r = 1.0f;
    marker->color.g = 1.0f;
    marker->color.b = 1.0f;
  }
  marker->lifetime = ros::Duration();
}

void laneletDirectionAsMarker(const lanelet::ConstLanelet ll, visualization_msgs::Marker* marker, const int id,
                              const std::string ns)
{
  if (marker == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": marker is null pointer!");
    return;
  }

  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time();
  marker->ns = ns;
  marker->id = id;
  marker->type = visualization_msgs::Marker::TRIANGLE_LIST;

  lanelet::BasicPoint3d pt[3];
  pt[0].x() = 0.0;
  pt[0].y() = -0.3;
  pt[0].z() = 0.0;
  pt[1].x() = 0.0;
  pt[1].y() = 0.3;
  pt[1].z() = 0;
  pt[2].x() = 1.0;
  pt[2].y() = 0.0;
  pt[2].z() = 0;

  lanelet::BasicPoint3d pc, pc2;

  lanelet::ConstLineString3d center_ls = ll.centerline();
  float s = 1.0;

  marker->lifetime = ros::Duration();

  marker->pose.position.x = 0.0;  // p.x();
  marker->pose.position.y = 0.0;  // p.y();
  marker->pose.position.z = 0.0;  // p.z();
  marker->scale.x = s;
  marker->scale.y = s;
  marker->scale.z = s;
  marker->color.r = 1.0f;
  marker->color.g = 1.0f;
  marker->color.b = 1.0f;
  marker->color.a = 1.0f;

  lanelet::Attribute attr = ll.attribute("turn_direction");
  double turn_dir = 0;

  std_msgs::ColorRGBA c;
  c.r = 0.0;
  c.g = 0.0;
  c.b = 1.0;
  c.a = 0.6;

  if (isLaneletAttributeValue(ll, "turn_direction", "right"))
  {
    turn_dir = -M_PI / 2.0;
    c.r = 1.0;
    c.g = 0.0;
    c.b = 1.0;
  }
  else if (isLaneletAttributeValue(ll, "turn_direction", "left"))
  {
    turn_dir = M_PI / 2.0;
    c.r = 0.0;
    c.g = 1.0;
    c.b = 1.0;
  }

  for (int ci = 0; ci < center_ls.size() - 1;)
  {
    pc = center_ls[ci];
    if (center_ls.size() > 1)
      pc2 = center_ls[ci + 1];
    else
      return;

    double heading = atan2(pc2.y() - pc.y(), pc2.x() - pc.x());

    lanelet::BasicPoint3d pt_tf[3];

    Eigen::Vector3d axis(0, 0, 1);
    Eigen::Transform<double, 3, Eigen::Affine> t(Eigen::AngleAxis<double>(heading, axis));

    for (int i = 0; i < 3; i++)
      pt_tf[i] = t * pt[i] + pc;

    geometry_msgs::Point gp[3];

    for (int i = 0; i < 3; i++)
    {
      std_msgs::ColorRGBA cn = c;

      gp[i].x = pt_tf[i].x();
      gp[i].y = pt_tf[i].y();
      gp[i].z = pt_tf[i].z();
      marker->points.push_back(gp[i]);
      marker->colors.push_back(cn);
    }
    ci = ci + 1;
  }
}

bool isReflexAngle(const geometry_msgs::Point32& a, const geometry_msgs::Point32& o, const geometry_msgs::Point32& b)
{
  // angle aob is reflex in counterclock wise direction if point b is on the right side of vector oa
  // which can be found out by calculating cross product of oa and ob
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x) < 0;
}

bool isWithinTriangle(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b, const geometry_msgs::Point32& c,
                      const geometry_msgs::Point32& p)
{
  // point p in within triangle if p is on the same side
  const double side1 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);  // a to b
  const double side2 = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);  // b to c
  const double side3 = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);  // c to a
  return (side1 > 0.0 && side2 > 0.0 && side3 > 0.0) || (side1 < 0.0 && side2 < 0.0 && side3 < 0.0);
}

}  // anonymous namespace

namespace lanelet
{
void visualization::lanelet2Triangle(const lanelet::ConstLanelet& ll, std::vector<geometry_msgs::Polygon>* triangles)
{
  if (triangles == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": triangles is null pointer!");
    return;
  }

  triangles->clear();
  geometry_msgs::Polygon ll_poly;
  lanelet2Polygon(ll, &ll_poly);
  polygon2Triangle(ll_poly, triangles);
}

void visualization::polygon2Triangle(const geometry_msgs::Polygon& polygon,
                                     std::vector<geometry_msgs::Polygon>* triangles)
{
  geometry_msgs::Polygon poly = polygon;
  // ear clipping: find smallest internal angle in polygon
  int N = poly.points.size();

  // array of angles for each vertex
  std::vector<bool> is_reflex_angle;
  is_reflex_angle.assign(N, false);
  for (int i = 0; i < N; i++)
  {
    geometry_msgs::Point32 p0, p1, p2;

    adjacentPoints(i, N, poly, &p0, &p1, &p2);
    is_reflex_angle.at(i) = isReflexAngle(p0, p1, p2);
  }

  // start ear clipping
  while (N >= 3)
  {
    int clipped_vertex = -1;

    for (int i = 0; i < N; i++)
    {
      bool is_reflex = is_reflex_angle.at(i);
      if (!is_reflex)
      {
        geometry_msgs::Point32 p0, p1, p2;
        adjacentPoints(i, N, poly, &p0, &p1, &p2);

        int j_begin = (i + 2) % N;
        int j_end = (i - 1 + N) % N;
        bool is_ear = true;
        for (int j = j_begin; j != j_end; j = (j + 1) % N)
        {
          if (isWithinTriangle(p0, p1, p2, poly.points.at(j)))
          {
            is_ear = false;
            break;
          }
        }

        if (is_ear)
        {
          clipped_vertex = i;
          break;
        }
      }
    }
    if (clipped_vertex < 0 || clipped_vertex >= N)
    {
      ROS_ERROR("Could not find valid vertex for ear clipping triangulation. Triangulation result might be invalid");
      clipped_vertex = 0;
    }

    // create triangle
    geometry_msgs::Point32 p0, p1, p2;
    adjacentPoints(clipped_vertex, N, poly, &p0, &p1, &p2);
    geometry_msgs::Polygon triangle;
    triangle.points.push_back(p0);
    triangle.points.push_back(p1);
    triangle.points.push_back(p2);
    triangles->push_back(triangle);

    // remove vertex of center of angle
    auto it = poly.points.begin();
    std::advance(it, clipped_vertex);
    poly.points.erase(it);

    // remove from angle list
    auto it_angle = is_reflex_angle.begin();
    std::advance(it_angle, clipped_vertex);
    is_reflex_angle.erase(it_angle);

    // update angle list
    N = poly.points.size();
    if (clipped_vertex == N)
    {
      clipped_vertex = 0;
    }
    adjacentPoints(clipped_vertex, N, poly, &p0, &p1, &p2);
    is_reflex_angle.at(clipped_vertex) = isReflexAngle(p0, p1, p2);

    int i_prev = (clipped_vertex == 0) ? N - 1 : clipped_vertex - 1;
    adjacentPoints(i_prev, N, poly, &p0, &p1, &p2);
    is_reflex_angle.at(i_prev) = isReflexAngle(p0, p1, p2);
  }
}

void visualization::lanelet2Polygon(const lanelet::ConstLanelet& ll, geometry_msgs::Polygon* polygon)
{
  if (polygon == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": polygon is null pointer!");
    return;
  }

  lanelet::CompoundPolygon3d ll_poly = ll.polygon3d();

  polygon->points.clear();
  polygon->points.reserve(ll_poly.size());

  for (const auto& pt : ll_poly)
  {
    geometry_msgs::Point32 pt32;
    utils::conversion::toGeomMsgPt32(pt.basicPoint(), &pt32);
    polygon->points.push_back(pt32);
  }
}

visualization_msgs::MarkerArray visualization::laneletDirectionAsMarkerArray(const lanelet::ConstLanelets lanelets)
{
  visualization_msgs::MarkerArray marker_array;
  int ll_dir_count = 0;
  for (auto lli = lanelets.begin(); lli != lanelets.end(); lli++)
  {
    lanelet::ConstLanelet ll = *lli;

    //      bool road_found = false;
    if (ll.hasAttribute(std::string("turn_direction")))
    {
      lanelet::Attribute attr = ll.attribute("turn_direction");
      visualization_msgs::Marker marker;

      laneletDirectionAsMarker(ll, &marker, ll_dir_count, "lanelet direction");

      marker_array.markers.push_back(marker);
      ll_dir_count++;
    }
  }

  return (marker_array);
}

visualization_msgs::MarkerArray visualization::autowareTrafficLightsAsMarkerArray(
    const std::vector<lanelet::AutowareTrafficLightConstPtr> tl_reg_elems, const std_msgs::ColorRGBA c,
    const ros::Duration duration, const double scale)
{
  visualization_msgs::MarkerArray tl_marker_array;

  int tl_count = 0;
  for (auto tli = tl_reg_elems.begin(); tli != tl_reg_elems.end(); tli++)
  {
    lanelet::ConstLineStrings3d light_bulbs;
    lanelet::AutowareTrafficLightConstPtr tl = *tli;

    const auto lights = tl->trafficLights();
    for (const auto& lsp : lights)
    {
      if (lsp.isLineString())  // traffic ligths can either polygons or
      {                        // linestrings
        lanelet::ConstLineString3d ls = static_cast<lanelet::ConstLineString3d>(lsp);

        visualization_msgs::Marker marker;
        visualization::trafficLight2TriangleMarker(ls, &marker, "traffic_light_triangle", c, duration, scale);
        tl_marker_array.markers.push_back(marker);
      }
    }

    light_bulbs = tl->lightBulbs();
    for (auto ls : light_bulbs)
    {
      lanelet::ConstLineString3d l = static_cast<lanelet::ConstLineString3d>(ls);

      for (auto pt : l)
      {
        if (pt.hasAttribute("color"))
        {
          visualization_msgs::Marker marker;
          lightAsMarker(pt, &marker, "traffic_light");
          tl_marker_array.markers.push_back(marker);

          tl_count++;
        }
      }
    }
  }

  return (tl_marker_array);
}

visualization_msgs::MarkerArray
visualization::lineStringsAsMarkerArray(const std::vector<lanelet::ConstLineString3d> line_strings,
                                        const std::string name_space, const std_msgs::ColorRGBA c, const double lss)
{
  visualization_msgs::MarkerArray ls_marker_array;
  for (auto i = line_strings.begin(); i != line_strings.end(); i++)
  {
    lanelet::ConstLineString3d ls = *i;
    visualization_msgs::Marker ls_marker;

    visualization::lineString2Marker(ls, &ls_marker, "map", name_space, c, 0.2);

    ls_marker_array.markers.push_back(ls_marker);
  }

  return (ls_marker_array);
}

visualization_msgs::MarkerArray visualization::laneletsBoundaryAsMarkerArray(const lanelet::ConstLanelets& lanelets,
                                                                             const std_msgs::ColorRGBA c,
                                                                             const bool viz_centerline)
{
  double lss = 0.2;  // line string size
  visualization_msgs::MarkerArray marker_array;
  for (auto li = lanelets.begin(); li != lanelets.end(); li++)
  {
    lanelet::ConstLanelet lll = *li;

    lanelet::ConstLineString3d left_ls = lll.leftBound();
    lanelet::ConstLineString3d right_ls = lll.rightBound();
    lanelet::ConstLineString3d center_ls = lll.centerline();

    visualization_msgs::Marker left_line_strip, right_line_strip, center_line_strip;

    visualization::lineString2Marker(left_ls, &left_line_strip, "map", "left_lane_bound", c, lss);
    visualization::lineString2Marker(right_ls, &right_line_strip, "map", "right_lane_bound", c, lss);
    marker_array.markers.push_back(left_line_strip);
    marker_array.markers.push_back(right_line_strip);
    if (viz_centerline)
    {
      visualization::lineString2Marker(center_ls, &center_line_strip, "map", "center_lane_line", c, lss * 0.5);
      marker_array.markers.push_back(center_line_strip);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray visualization::trafficLightsAsTriangleMarkerArray(
    const std::vector<lanelet::TrafficLightConstPtr> tl_reg_elems, const std_msgs::ColorRGBA c,
    const ros::Duration duration, const double scale)
{
  // convert to to an array of linestrings and publish as marker array using
  // exisitng function

  int tl_count = 0;
  std::vector<lanelet::ConstLineString3d> line_strings;
  visualization_msgs::MarkerArray marker_array;

  for (auto tli = tl_reg_elems.begin(); tli != tl_reg_elems.end(); tli++)
  {
    lanelet::TrafficLightConstPtr tl = *tli;
    lanelet::LineString3d ls;

    auto lights = tl->trafficLights();
    for (auto lsp : lights)
    {
      if (lsp.isLineString())  // traffic ligths can either polygons or
      {                        // linestrings
        lanelet::ConstLineString3d ls = static_cast<lanelet::ConstLineString3d>(lsp);

        visualization_msgs::Marker marker;
        visualization::trafficLight2TriangleMarker(ls, &marker, "traffic_light_triangle", c, duration, scale);
        marker_array.markers.push_back(marker);
        tl_count++;
      }
    }
  }

  return (marker_array);
}

visualization_msgs::MarkerArray visualization::laneletsAsTriangleMarkerArray(const std::string ns,
                                                                             const lanelet::ConstLanelets& lanelets,
                                                                             const std_msgs::ColorRGBA c)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.lifetime = ros::Duration();
  marker.pose.position.x = 0.0;  // p.x();
  marker.pose.position.y = 0.0;  // p.y();
  marker.pose.position.z = 0.0;  // p.z();
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;

  for (auto ll : lanelets)
  {
    std::vector<geometry_msgs::Polygon> triangles;
    lanelet2Triangle(ll, &triangles);

    for (auto tri : triangles)
    {
      geometry_msgs::Point tri0[3];

      for (int i = 0; i < 3; i++)
      {
        utils::conversion::toGeomMsgPt(tri.points[i], &tri0[i]);

        marker.points.push_back(tri0[i]);
        marker.colors.push_back(c);
      }
    }
  }
  if (!marker.points.empty())
  {
    marker_array.markers.push_back(marker);
  }

  return (marker_array);
}

void visualization::trafficLight2TriangleMarker(const lanelet::ConstLineString3d ls, visualization_msgs::Marker* marker,
                                                const std::string ns, const std_msgs::ColorRGBA cl,
                                                const ros::Duration duration, const double scale)
{
  if (marker == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": marker is null pointer!");
    return;
  }
  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time();
  marker->ns = ns;
  marker->id = ls.id();
  marker->type = visualization_msgs::Marker::TRIANGLE_LIST;

  marker->lifetime = duration;

  marker->pose.position.x = 0.0;  // p.x();
  marker->pose.position.y = 0.0;  // p.y();
  marker->pose.position.z = 0.0;  // p.z();
  marker->scale.x = 1.0;
  marker->scale.y = 1.0;
  marker->scale.z = 1.0;
  marker->color.r = 1.0f;
  marker->color.g = 1.0f;
  marker->color.b = 1.0f;
  marker->color.a = 1.0f;

  double h = 0.7;
  if (ls.hasAttribute("height"))
  {
    lanelet::Attribute attr = ls.attribute("height");
    h = std::stod(attr.value());
  }

  // construct triangles and add to marker

  // define polygon of traffic light border
  Eigen::Vector3d v[4];
  v[0] << ls.front().x(), ls.front().y(), ls.front().z();
  v[1] << ls.back().x(), ls.back().y(), ls.back().z();
  v[2] << ls.back().x(), ls.back().y(), ls.back().z() + h;
  v[3] << ls.front().x(), ls.front().y(), ls.front().z() + h;

  Eigen::Vector3d c = (v[0] + v[1] + v[2] + v[3]) / 4;

  if (scale > 0.0 && scale != 1.0)
  {
    for (int i = 0; i < 4; i++)
    {
      v[i] = (v[i] - c) * scale + c;
    }
  }
  geometry_msgs::Point tri0[3];
  utils::conversion::toGeomMsgPt(v[0], &tri0[0]);
  utils::conversion::toGeomMsgPt(v[1], &tri0[1]);
  utils::conversion::toGeomMsgPt(v[2], &tri0[2]);
  geometry_msgs::Point tri1[3];
  utils::conversion::toGeomMsgPt(v[0], &tri1[0]);
  utils::conversion::toGeomMsgPt(v[2], &tri1[1]);
  utils::conversion::toGeomMsgPt(v[3], &tri1[2]);

  for (int i = 0; i < 3; i++)
  {
    marker->points.push_back(tri0[i]);
    marker->colors.push_back(cl);
  }
  for (int i = 0; i < 3; i++)
  {
    marker->points.push_back(tri1[i]);
    marker->colors.push_back(cl);
  }
}

void visualization::lineString2Marker(const lanelet::ConstLineString3d ls, visualization_msgs::Marker* line_strip,
                                      const std::string frame_id, const std::string ns, const std_msgs::ColorRGBA c,
                                      const float lss)
{
  if (line_strip == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": line_strip is null pointer!");
    return;
  }

  line_strip->header.frame_id = frame_id;
  line_strip->header.stamp = ros::Time();
  line_strip->ns = ns;
  line_strip->action = visualization_msgs::Marker::ADD;

  line_strip->pose.orientation.w = 1.0;
  line_strip->id = ls.id();

  line_strip->type = visualization_msgs::Marker::LINE_STRIP;

  line_strip->scale.x = lss;

  line_strip->color = c;

  // fill out lane line
  for (auto i = ls.begin(); i != ls.end(); i++)
  {
    geometry_msgs::Point p;
    p.x = (*i).x();
    p.y = (*i).y();
    p.z = (*i).z();
    line_strip->points.push_back(p);
  }
}

}  // namespace lanelet
