/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
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
 */

#include "op_planner/PlanningHelpers.h"

using namespace PlannerHNS;

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <vector>

class TestSuite:
  public ::testing::Test
{
public:
  TestSuite() {}
};

TEST(TestSuite, TestNoPathFound)
{
  WayPoint start_position;
  WayPoint intermediate_position_1;
  WayPoint intermediate_position_2;
  WayPoint intermediate_position_3;
  WayPoint goal_position;
  WayPoint* result;
  std::vector<int> global_path;
  double distance_limit = 34;
  bool enable_line_change = false;
  std::vector<WayPoint*> all_cell_to_delete;
  double fallback_min_goal_distance_th = 0;

  start_position.pos.x = 0;
  start_position.pos.y = 0;
  start_position.pos.z = 0;
  start_position.pos.a = 0;
  start_position.pFronts.push_back(&intermediate_position_1);

  intermediate_position_1.pos.x = 10;
  intermediate_position_1.pos.y = 0;
  intermediate_position_1.pos.z = 0;
  intermediate_position_1.pos.a = 0;
  intermediate_position_1.pFronts.push_back(&intermediate_position_2);
  intermediate_position_1.pBacks.push_back(&start_position);
  intermediate_position_1.id = 1;

  intermediate_position_2.pos.x = 20;
  intermediate_position_2.pos.y = 0;
  intermediate_position_2.pos.z = 0;
  intermediate_position_2.pos.a = 0;
  intermediate_position_2.pFronts.push_back(&intermediate_position_3);
  intermediate_position_2.pBacks.push_back(&intermediate_position_1);
  intermediate_position_2.id = 2;

  intermediate_position_3.pos.x = 30;
  intermediate_position_3.pos.y = 0;
  intermediate_position_3.pos.z = 0;
  intermediate_position_3.pos.a = 0;
  intermediate_position_3.pBacks.push_back(&intermediate_position_2);
  intermediate_position_3.id = 3;

  goal_position.pos.x = 34;
  goal_position.pos.y = 0;
  goal_position.pos.z = 0;
  goal_position.pos.a = 0;

  result = PlanningHelpers::BuildPlanningSearchTreeV2(
    &start_position,
    goal_position,
    global_path,
    distance_limit,
    enable_line_change,
    all_cell_to_delete,
    fallback_min_goal_distance_th);

  // Assert no path is found
  ASSERT_TRUE(result == 0);
}

TEST(TestSuite, TestPathFound)
{
  WayPoint start_position;
  WayPoint intermediate_position_1;
  WayPoint intermediate_position_2;
  WayPoint intermediate_position_3;
  WayPoint goal_position;
  WayPoint* result;
  std::vector<int> global_path;
  double distance_limit = 34;
  bool enable_line_change = false;
  std::vector<WayPoint*> all_cell_to_delete;
  double fallback_min_goal_distance_th = 0;

  start_position.pos.x = 0;
  start_position.pos.y = 0;
  start_position.pos.z = 0;
  start_position.pos.a = 0;
  start_position.pFronts.push_back(&intermediate_position_1);

  intermediate_position_1.pos.x = 10;
  intermediate_position_1.pos.y = 0;
  intermediate_position_1.pos.z = 0;
  intermediate_position_1.pos.a = 0;
  intermediate_position_1.pFronts.push_back(&intermediate_position_2);
  intermediate_position_1.pBacks.push_back(&start_position);
  intermediate_position_1.id = 1;

  intermediate_position_2.pos.x = 20;
  intermediate_position_2.pos.y = 0;
  intermediate_position_2.pos.z = 0;
  intermediate_position_2.pos.a = 0;
  intermediate_position_2.pFronts.push_back(&intermediate_position_3);
  intermediate_position_2.pBacks.push_back(&intermediate_position_1);
  intermediate_position_2.id = 2;

  intermediate_position_3.pos.x = 34;
  intermediate_position_3.pos.y = 0;
  intermediate_position_3.pos.z = 0;
  intermediate_position_3.pos.a = 0;
  intermediate_position_3.pBacks.push_back(&intermediate_position_2);
  intermediate_position_3.id = 3;

  goal_position.pos.x = 34;
  goal_position.pos.y = 0;
  goal_position.pos.z = 0;
  goal_position.pos.a = 0;

  result = PlanningHelpers::BuildPlanningSearchTreeV2(
    &start_position,
    goal_position,
    global_path,
    distance_limit,
    enable_line_change,
    all_cell_to_delete,
    fallback_min_goal_distance_th);

  // Assert a path is found
  ASSERT_TRUE(result != 0);
  ASSERT_DOUBLE_EQ(result->pos.x, 34);
  ASSERT_DOUBLE_EQ(result->pos.y, 0);
  ASSERT_DOUBLE_EQ(result->pos.z, 0);
  ASSERT_DOUBLE_EQ(result->pos.a, 0);
}

TEST(TestSuite, TestPathCloseEnough)
{
  WayPoint start_position;
  WayPoint intermediate_position_1;
  WayPoint intermediate_position_2;
  WayPoint intermediate_position_3;
  WayPoint goal_position;
  WayPoint* result;
  std::vector<int> global_path;
  double distance_limit = 34;
  bool enable_line_change = false;
  std::vector<WayPoint*> all_cell_to_delete;
  double fallback_min_goal_distance_th = 5;

  start_position.pos.x = 0;
  start_position.pos.y = 0;
  start_position.pos.z = 0;
  start_position.pos.a = 0;
  start_position.pFronts.push_back(&intermediate_position_1);

  intermediate_position_1.pos.x = 10;
  intermediate_position_1.pos.y = 0;
  intermediate_position_1.pos.z = 0;
  intermediate_position_1.pos.a = 0;
  intermediate_position_1.pFronts.push_back(&intermediate_position_2);
  intermediate_position_1.pBacks.push_back(&start_position);
  intermediate_position_1.id = 1;

  intermediate_position_2.pos.x = 20;
  intermediate_position_2.pos.y = 0;
  intermediate_position_2.pos.z = 0;
  intermediate_position_2.pos.a = 0;
  intermediate_position_2.pFronts.push_back(&intermediate_position_3);
  intermediate_position_2.pBacks.push_back(&intermediate_position_1);
  intermediate_position_2.id = 2;

  intermediate_position_3.pos.x = 30;
  intermediate_position_3.pos.y = 0;
  intermediate_position_3.pos.z = 0;
  intermediate_position_3.pos.a = 0;
  intermediate_position_3.pBacks.push_back(&intermediate_position_2);
  intermediate_position_3.id = 3;

  goal_position.pos.x = 34;
  goal_position.pos.y = 0;
  goal_position.pos.z = 0;
  goal_position.pos.a = 0;

  result = PlanningHelpers::BuildPlanningSearchTreeV2(
    &start_position,
    goal_position,
    global_path,
    distance_limit,
    enable_line_change,
    all_cell_to_delete,
    fallback_min_goal_distance_th);

  // Assert a close enough path is found
  ASSERT_TRUE(result != 0);
  ASSERT_DOUBLE_EQ(result->pos.x, 30);
  ASSERT_DOUBLE_EQ(result->pos.y, 0);
  ASSERT_DOUBLE_EQ(result->pos.z, 0);
  ASSERT_DOUBLE_EQ(result->pos.a, 0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}