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

#include <lanelet2_projection/UTM.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_core/LaneletMap.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <lanelet2_extension/utility/utilities.h>

#include <autoware_lanelet2_msgs/MapBin.h>

#include <string>
#include <boost/filesystem.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lanelet_map_loader");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string lanelet2_path;
  pnh.param<std::string>("lanelet2_path", lanelet2_path, "");

  std::string lanelet2_file_path;
  boost::filesystem::path path(lanelet2_path);
  if (boost::filesystem::is_regular_file(path))
  {
    // If file
    lanelet2_file_path = path.generic_string();
  }
  else if (boost::filesystem::is_directory(path))
  {
    // If directory
    std::vector<boost::filesystem::path> file_path_list;
    for (const boost::filesystem::path& entry :
         boost::make_iterator_range(boost::filesystem::directory_iterator(path), {}))
    {
      if (boost::filesystem::is_regular_file(entry))
      {
        file_path_list.push_back(entry);
      }
    }

    if (file_path_list.size() > 0)
    {
      // Find first path
      auto min_it = std::min_element(file_path_list.begin(), file_path_list.end(),
                                     [](const boost::filesystem::path& a, const boost::filesystem::path& b) {
                                       return a.filename().generic_string() < b.filename().generic_string();
                                     });
      lanelet2_file_path = (*min_it).generic_string();
    }
    else
    {
      lanelet2_file_path = "";
    }
  }

  if (lanelet2_file_path == "")
  {
    ROS_ERROR("[lanelet2_map_loader] File name is not specified or wrong. [%s]", lanelet2_file_path.c_str());
    return EXIT_FAILURE;
  }

  ROS_INFO("[lanelet2_map_loader] Will load %s", lanelet2_file_path.c_str());

  lanelet::ErrorMessages errors;

  lanelet::projection::MGRSProjector projector;
  lanelet::LaneletMapPtr map = lanelet::load(lanelet2_file_path, projector, &errors);

  for (const auto& error : errors)
  {
    ROS_ERROR_STREAM(error);
  }
  if (!errors.empty())
  {
    return EXIT_FAILURE;
  }

  lanelet::utils::overwriteLaneletsCenterline(map, false);

  std::string format_version, map_version;
  lanelet::io_handlers::AutowareOsmParser::parseVersions(lanelet2_file_path, &format_version, &map_version);

  ros::Publisher map_bin_pub = nh.advertise<autoware_lanelet2_msgs::MapBin>("/lanelet_map_bin", 1, true);
  autoware_lanelet2_msgs::MapBin map_bin_msg;
  map_bin_msg.header.stamp = ros::Time::now();
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.format_version = format_version;
  map_bin_msg.map_version = map_version;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  map_bin_pub.publish(map_bin_msg);

  ros::spin();

  return 0;
}
