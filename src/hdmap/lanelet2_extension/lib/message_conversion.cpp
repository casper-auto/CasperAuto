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

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/io_handlers/OsmFile.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_projection/UTM.h>
#include <ros/ros.h>

#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/utility/message_conversion.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <sstream>
#include <string>

namespace lanelet
{
namespace utils
{
namespace conversion
{
void toBinMsg(const lanelet::LaneletMapPtr& map, autoware_lanelet2_msgs::MapBin* msg)
{
  if (msg == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "msg is null pointer!");
    return;
  }

  std::stringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << *map;
  auto id_counter = lanelet::utils::getId();
  oa << id_counter;

  std::string data_str(ss.str());

  msg->data.clear();
  msg->data.assign(data_str.begin(), data_str.end());
}

void fromBinMsg(const autoware_lanelet2_msgs::MapBin& msg, lanelet::LaneletMapPtr map)
{
  if (!map)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": map is null pointer!");
    return;
  }

  std::string data_str;
  data_str.assign(msg.data.begin(), msg.data.end());

  std::stringstream ss;
  ss << data_str;
  boost::archive::binary_iarchive oa(ss);
  oa >> *map;
  lanelet::Id id_counter;
  oa >> id_counter;
  lanelet::utils::registerId(id_counter);
  // *map = std::move(laneletMap);
}

void toGeomMsgPt(const geometry_msgs::Point32& src, geometry_msgs::Point* dst)
{
  if (dst == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "pointer is null!");
    return;
  }
  dst->x = src.x;
  dst->y = src.y;
  dst->z = src.z;
}
void toGeomMsgPt(const Eigen::Vector3d& src, geometry_msgs::Point* dst)
{
  if (dst == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "pointer is null!");
    return;
  }
  dst->x = src.x();
  dst->y = src.y();
  dst->z = src.z();
}
void toGeomMsgPt(const lanelet::ConstPoint3d& src, geometry_msgs::Point* dst)
{
  if (dst == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "pointer is null!");
    return;
  }
  dst->x = src.x();
  dst->y = src.y();
  dst->z = src.z();
}
void toGeomMsgPt(const lanelet::ConstPoint2d& src, geometry_msgs::Point* dst)
{
  if (dst == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "pointer is null!");
    return;
  }
  dst->x = src.x();
  dst->y = src.y();
  dst->z = 0;
}

void toGeomMsgPt32(const Eigen::Vector3d& src, geometry_msgs::Point32* dst)
{
  if (dst == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "pointer is null!");
    return;
  }
  dst->x = src.x();
  dst->y = src.y();
  dst->z = src.z();
}

geometry_msgs::Point toGeomMsgPt(const geometry_msgs::Point32& src)
{
  geometry_msgs::Point dst;
  toGeomMsgPt(src, &dst);
  return dst;
}
geometry_msgs::Point toGeomMsgPt(const Eigen::Vector3d& src)
{
  geometry_msgs::Point dst;
  toGeomMsgPt(src, &dst);
  return dst;
}
geometry_msgs::Point toGeomMsgPt(const lanelet::ConstPoint3d& src)
{
  geometry_msgs::Point dst;
  toGeomMsgPt(src, &dst);
  return dst;
}
geometry_msgs::Point toGeomMsgPt(const lanelet::ConstPoint2d& src)
{
  geometry_msgs::Point dst;
  toGeomMsgPt(src, &dst);
  return dst;
}

}  // namespace conversion
}  // namespace utils
}  // namespace lanelet
