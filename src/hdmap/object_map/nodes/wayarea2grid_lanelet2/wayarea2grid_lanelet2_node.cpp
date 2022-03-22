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
#include <ros/ros.h>
#include <wayarea2grid_lanelet2/wayarea2grid_lanelet2.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wayarea2grid_lanelet2");
  object_map::WayareaToGridLanelet2 wayarea2grid;

  wayarea2grid.Run();

  return 0;
}
