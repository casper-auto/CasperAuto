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
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <pure_pursuit/pure_pursuit_core.h>

namespace waypoint_follower
{
class PurePursuitNodeTestSuite : public ::testing::Test
{
protected:
  std::unique_ptr<PurePursuitNode> obj_;
  virtual void SetUp()
  {
    obj_ = std::unique_ptr<PurePursuitNode>(new PurePursuitNode());
    obj_->add_virtual_end_waypoints_ = true;
  }
  virtual void TearDown()
  {
    obj_.reset();
  }

public:
  PurePursuitNodeTestSuite() = default;
  ~PurePursuitNodeTestSuite() = default;
  WaypointArrayDirection getDirection()
  {
    return obj_->direction_;
  }
  void ppCallbackFromWayPoints(const casper_auto_msgs::WaypointArrayConstPtr& msg)
  {
    obj_->callbackFromWayPoints(msg);
  }
  void ppConnectVirtualLastWaypoints(casper_auto_msgs::WaypointArray* expand_waypoint_array, WaypointArrayDirection direction)
  {
    obj_->connectVirtualLastWaypoints(expand_waypoint_array, direction);
  }
};

TEST_F(PurePursuitNodeTestSuite, inputPositivePath)
{
  casper_auto_msgs::WaypointArray original_waypoint_array;
  original_waypoint_array.waypoints.resize(3, autoware_msgs::Waypoint());
  for (int i = 0; i < 3; i++)
  {
    original_waypoint_array.waypoints[i].pose.pose.position.x = i;
    original_waypoint_array.waypoints[i].pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  }
  const casper_auto_msgs::WaypointArrayConstPtr lp(boost::make_shared<casper_auto_msgs::WaypointArray>(original_waypoint_array));
  ppCallbackFromWayPoints(lp);
  ASSERT_EQ(getDirection(), WaypointArrayDirection::Forward) << "direction is not matching to positive waypoint_array.";
}

TEST_F(PurePursuitNodeTestSuite, inputNegativePath)
{
  casper_auto_msgs::WaypointArray original_waypoint_array;
  original_waypoint_array.waypoints.resize(3, autoware_msgs::Waypoint());
  for (int i = 0; i < 3; i++)
  {
    original_waypoint_array.waypoints[i].pose.pose.position.x = -i;
    original_waypoint_array.waypoints[i].pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  }
  const casper_auto_msgs::WaypointArrayConstPtr lp(boost::make_shared<casper_auto_msgs::WaypointArray>(original_waypoint_array));
  ppCallbackFromWayPoints(lp);
  ASSERT_EQ(getDirection(), WaypointArrayDirection::Backward) << "direction is not matching to negative waypoint_array.";
}
// If original waypoint_array is empty, new waypoint_array is also empty.
TEST_F(PurePursuitNodeTestSuite, inputEmptyWaypointArray)
{
  casper_auto_msgs::WaypointArray original_waypoint_array, new_waypoint_array;
  ppConnectVirtualLastWaypoints(&new_waypoint_array, WaypointArrayDirection::Forward);
  ASSERT_EQ(original_waypoint_array.waypoints.size(), new_waypoint_array.waypoints.size()) << "Input empty waypoint_array, and output is not empty";
}

// If the original waypoint_array exceeds 2 points,
// the additional part will be updated at
// the interval of the first 2 points.
TEST_F(PurePursuitNodeTestSuite, inputNormalWaypointArray)
{
  casper_auto_msgs::WaypointArray original_waypoint_array;
  original_waypoint_array.waypoints.resize(2, autoware_msgs::Waypoint());
  for (int i = 0; i < 2; i++)
  {
    original_waypoint_array.waypoints[i].pose.pose.position.x = i;
  }
  casper_auto_msgs::WaypointArray new_waypoint_array(original_waypoint_array);
  ppConnectVirtualLastWaypoints(&new_waypoint_array, WaypointArrayDirection::Forward);

  ASSERT_LT(original_waypoint_array.waypoints.size(), new_waypoint_array.waypoints.size()) << "Fail to expand waypoints";
}
}  // namespace waypoint_follower

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PurePursuitTest");
  return RUN_ALL_TESTS();
}
