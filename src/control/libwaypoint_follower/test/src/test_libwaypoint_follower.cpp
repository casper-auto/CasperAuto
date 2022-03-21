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

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "libwaypoint_follower/test_libwaypoint_follower.h"

class LibWaypointFollowerTestSuite :
  public ::testing::Test
{
public:
  LibWaypointFollowerTestSuite() {}
  LibWaypointFollowerTestClass test_obj_;

protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

TEST_F(LibWaypointFollowerTestSuite, calcRelativeCoordinate)
{
//  The member variable x of calcRelativeCoordinate must return following:
//  - if      target_point is in front of current_pose,  return positive value
//  - else if target_point is behind from current_pose,  return negative value
//  - else    (target_point == current_pose),            return 0
  auto point = [](double x, double y, double z)
  {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return std::move(p);
  };
  const geometry_msgs::PoseStamped cpos = test_obj_.generateCurrentPose(0, 0, 0);
  std::map<std::string, std::pair<geometry_msgs::Point, CoordinateResult>> dataset;
  dataset["(is_forward)"] = std::make_pair(point(1, 0, 0), CoordinateResult::Positive);
  dataset["(is_backward)"] = std::make_pair(point(-1, 0, 0), CoordinateResult::Negative);
  dataset["(is_equal)"] = std::make_pair(point(0, 0, 0), CoordinateResult::Equal);
  for (const auto& el : dataset)
  {
    const double x = calcRelativeCoordinate(el.second.first, cpos.pose).x;
    const CoordinateResult ret =
      (x < 0.0) ? CoordinateResult::Negative
      : (x > 0.0) ? CoordinateResult::Positive : CoordinateResult::Equal;
    ASSERT_EQ(ret, el.second.second)
      << "Failure in " << el.first << ", it must be " << static_cast<int>(el.second.second) << ".";
  }
}

TEST_F(LibWaypointFollowerTestSuite, getWaypointArrayDirectionByPosition)
{
//  getWaypointArrayDirectionByPosition must return following:
//  - if      waypoint[i+1] is in front of waypoint[i],  return Forward
//  - else if waypoint[i+1] is behind from waypoint[i],  return Backward
//  - else if waypoint[i+1] == waypoint[i] in all i,     return Error
//  - else if waypoint size < 2,                         return Error
  std::map<std::string, std::pair<casper_auto_msgs::WaypointArray, WaypointArrayDirection>> dataset;
  dataset["(pos>0)"] = std::make_pair(test_obj_.generateWaypointArray(1, 5.0), WaypointArrayDirection::Forward);
  dataset["(pos<0)"] = std::make_pair(test_obj_.generateWaypointArray(-1, -5.0), WaypointArrayDirection::Backward);
  dataset["(pos=0)"] = std::make_pair(test_obj_.generateWaypointArray(0, -5.0), WaypointArrayDirection::Error);
  dataset["(size<2)"] = std::make_pair(test_obj_.generateOffsetWaypointArray(0, -5.0, 0.0, 0), WaypointArrayDirection::Error);
  for (const auto& el : dataset)
  {
    ASSERT_EQ(getWaypointArrayDirectionByPosition(el.second.first), el.second.second)
      << "Failure in " << el.first << ", it must be " << static_cast<int>(el.second.second) << ".";
  }
}

TEST_F(LibWaypointFollowerTestSuite, getWaypointArrayDirectionByVelocity)
{
//  getWaypointArrayDirectionByVelocity must return following:
//  - if waypoint[i+1] is in front of waypoint[i],      return Forward
//  - else if waypoint[i+1] is behind from waypoint[i], return Backward
//  - else (for all i: velocity[i] == 0),               return Error
  std::map<std::string, std::pair<casper_auto_msgs::WaypointArray, WaypointArrayDirection>> dataset;
  dataset["(vel>0)"] = std::make_pair(test_obj_.generateWaypointArray(1, 5.0), WaypointArrayDirection::Forward);
  dataset["(vel<0)"] = std::make_pair(test_obj_.generateWaypointArray(-1, -5.0), WaypointArrayDirection::Backward);
  dataset["(vel=0)"] = std::make_pair(test_obj_.generateWaypointArray(-1, 0.0), WaypointArrayDirection::Error);
  for (const auto& el : dataset)
  {
    ASSERT_EQ(getWaypointArrayDirectionByVelocity(el.second.first), el.second.second)
      << "Failure in " << el.first << ", it must be " << static_cast<int>(el.second.second) << ".";
  }
}

TEST_F(LibWaypointFollowerTestSuite, getWaypointArrayDirection)
{
//  getWaypointArrayDirection must return following:
//   Now, Rp means position identification result,
//    and Rv means velocity identification result.
//    Rp and Rv return Forward, Backward, or Error.
//  - if Rp != Rv and Rp != Error and Rv != Error,    return Error
//  - else if Rp != Error and Rv != Error,            return Rp(== Rv)
//    => if Rp == Forward and Rv == Forward(v > 0.0), return Forward
//  - else if Rp != Error and Rv == Error,            return Rp
//    => if Rp == Forward and Rv == Error,            return Forward
//  - else if Rp == Error and Rv != Error,            return Rv
//    => if Rp == Error and Rv == Forward(v > 0.0),   return Forward
//  - else (Rp == Error and Rv == Error),             return Rv(== ERROR)
  std::map<std::string, std::pair<casper_auto_msgs::WaypointArray, WaypointArrayDirection>> dataset;
  dataset["(Rp!=Rv)"] = std::make_pair(test_obj_.generateWaypointArray(1, -5.0), WaypointArrayDirection::Error);
  dataset["(Rp!=Err,Rv!=Err)"] = std::make_pair(test_obj_.generateWaypointArray(1, 5.0), WaypointArrayDirection::Forward);
  dataset["(Rp!=Err,Rv==Err)"] = std::make_pair(test_obj_.generateWaypointArray(1, 0.0), WaypointArrayDirection::Forward);
  dataset["(Rp==Err,Rv!=Err)"] = std::make_pair(test_obj_.generateWaypointArray(0, 5.0), WaypointArrayDirection::Forward);
  dataset["(Rp==Err,Rv==Err)"] = std::make_pair(test_obj_.generateWaypointArray(0, 0.0), WaypointArrayDirection::Error);
  for (const auto& el : dataset)
  {
    ASSERT_EQ(getWaypointArrayDirection(el.second.first), el.second.second)
      << "Failure in " << el.first << ", it must be " << static_cast<int>(el.second.second) << ".";
  }
}

TEST_F(LibWaypointFollowerTestSuite, inDrivingDirection)
{
//  inDrivingDirection must return following:
//   Now, Rd means waypoint_array direction identification result,
//    and Rc means relative Coordinate result.
//    Rd return Forward, Backward, or Error.
//    The menber variable x of Rc return double value.
//
//   case) current_pose == (0,0,0) && waypoint_array.x={-1, 0, 1, 2, ...}
//    Now, idx means the target waypoint_array index,
//      and vel means waypoint_array.velocity(constant value in all point)
//  - if Rd == Error,                   return false
//    =>if idx == Any and vel < 0.0,    return false
// - else if Rc.x > 0.0,                return (Rd == Forward)
//   => if idx > 1 and vel >=0.0,       return true
// - else if Rc.x = 0.0,                return (Rd == Forward)
//   => if idx == 1 and vel >= 0.0,     return true
// - else (Rc.x < 0.0),                 return (Rd == Backward)
//   => if idx < 1 and vel >= 0.0,      return false
  geometry_msgs::PoseStamped pose = test_obj_.generateCurrentPose(0, 0, 0);
  std::map<std::string, std::pair<DirectionCheckDataSet, bool>> dataset;
  dataset["(idx==Any,vel<0.0)"] = std::make_pair(DirectionCheckDataSet(0, -5.0), false);
  dataset["(idx>1,vel>=0.0)"] = std::make_pair(DirectionCheckDataSet(2, 5.0), true);
  dataset["(idx==1,vel>=0.0)"] = std::make_pair(DirectionCheckDataSet(1, 5.0), true);
  dataset["(idx<1,vel>=0.0)"] = std::make_pair(DirectionCheckDataSet(0, 5.0), false);
  for (const auto& el : dataset)
  {
    WayPoints wp;
    wp.setPath(test_obj_.generateOffsetWaypointArray(1, el.second.first.vel, -1.0, 100));
    bool ret = wp.inDrivingDirection(el.second.first.idx, pose.pose);
    ASSERT_EQ(ret, el.second.second)
      << "Failure in " + el.first << ", it must be " << el.second.second << ".";
  }
}


TEST_F(LibWaypointFollowerTestSuite, getClosestWaypoint)
{
//
// getClosestWaypoint must return following:
//   case) current_pose == (0,0,0)
// - if conflict path input,                                           return -1
// - else if no points path input,                                     return -1
// - else if success to search valid front near points,  return nearest idx(>0)
//   ("valid" means within distance and angle threshold)
//   =>  if valid_forward: lane_x = {-0.5,0.5,1.5,...} and vel >= 0,   return 1
//       if valid backward:lane_x = {1.5,0.5,-0.5,...} and vel < 0,    return 2
// - else if success to search invalid front near points, return nearest idx(>0)
//   =>  if over distance: lane_x = {6,7,8,9,...} and vel >= 0,        return 1
//       if opposite waypoint_array:
//         case) current_pose == (0,0,pi/2)
//                         lane_x = {-0.5,0.5,1.5,...} and vel >= 0,   return 1
// - else (fail to search front points),                               return -1
//   =>  if pass endpoint: lane_x = {-100,...,-1} and vel >= 0,        return -1

  geometry_msgs::PoseStamped valid_pose = test_obj_.generateCurrentPose(0, 0, 0);
  geometry_msgs::PoseStamped invalid_pose = test_obj_.generateCurrentPose(0, 0, M_PI / 2.0);
  std::map<std::string, std::pair<ClosestCheckDataSet, int>> dataset;
  dataset["(conflict_path)"] = std::make_pair(ClosestCheckDataSet(1, -5.0, 0.0, 100, valid_pose), -1);
  dataset["(no_point_path)"] = std::make_pair(ClosestCheckDataSet(1, 5.0, 0.0, 0, valid_pose), -1);
  dataset["(valid_forward)"] = std::make_pair(ClosestCheckDataSet(1, 5.0, -0.5, 100, valid_pose), 1);
  dataset["(valid_backward)"] = std::make_pair(ClosestCheckDataSet(-1, -5.0, -1.5, 100, valid_pose), 2);
  dataset["(over_distance)"] = std::make_pair(ClosestCheckDataSet(1, 5.0, 6.0, 100, valid_pose), 0);
  dataset["(opposite_lane)"] = std::make_pair(ClosestCheckDataSet(1, 5.0, -0.5, 100, invalid_pose), 1);
  dataset["(pass_endpoint)"] = std::make_pair(ClosestCheckDataSet(1, 5.0, -100.0, 100, valid_pose), -1);

  for (const auto& el : dataset)
  {
    const ClosestCheckDataSet& data = el.second.first;
    const auto& waypoint_array = test_obj_.generateOffsetWaypointArray(data.dir, data.vel, data.offset, data.num);
    int ret = getClosestWaypoint(waypoint_array, data.pose.pose);
    ASSERT_EQ(ret, el.second.second)
      << "Failure in " << el.first << ", it must be " << el.second.second << ".";
  }
}

TEST_F(LibWaypointFollowerTestSuite, calcCurvature)
{
  geometry_msgs::Point target;
  geometry_msgs::Pose curr_pose;

  target.x = 10.0;
  target.y = 0.0;
  ASSERT_NEAR(0.0, calcCurvature(target, curr_pose), ERROR);

  target.x = 10.0;
  target.y = 10.0;
  ASSERT_NEAR(0.1, calcCurvature(target, curr_pose), ERROR);

  target.x = 0.0;
  target.y = 10.0;
  ASSERT_NEAR(0.2, calcCurvature(target, curr_pose), ERROR);
}

TEST_F(LibWaypointFollowerTestSuite, calcDistSquared2D)
{
  geometry_msgs::Point p;
  geometry_msgs::Point q;

  // p.x = 0.0, p.y = 0.0, q.x = 0.0, q.y = 0.0, length2=0.0m
  ASSERT_NEAR(0.000000, calcDistSquared2D(p, q), ERROR);

  // p.x = 0.0, p.y = 0.0, p.z = 10.0, q.x = 0.0, q.y = 0.0, length2=0.0m, z is invalid value
  p.z = 10.0;
  ASSERT_NEAR(0.000000, calcDistSquared2D(p, q), ERROR);

  // p.x = 1.0, p.y = 0.0, q.x = 0.0, q.y = 0.0, length2=0.0m
  p.x = 1.0;
  p.z = 0.0;
  ASSERT_NEAR(1.000000, calcDistSquared2D(p, q), ERROR);

  // p.x = 2.0, p.y = 6.0, q.x = 6.0, q.y = 3.0, length2=25.0m
  p.x = 2.0;
  p.y = 6.0;
  q.x = 6.0;
  q.y = 3.0;
  ASSERT_NEAR(25.000000, calcDistSquared2D(p, q), ERROR);
}

TEST_F(LibWaypointFollowerTestSuite, calcLateralError)
{
  geometry_msgs::Point line_s;
  geometry_msgs::Point line_e;
  geometry_msgs::Point point;

  // target point is on right side of the line
  line_s.x = 2.0;
  line_s.y = 4.0;

  line_e.x = 7.0;
  line_e.y = 3.0;

  point.x = 6.0;
  point.y = -3.0;
  ASSERT_NEAR(-6.079600, calcLateralError2D(line_s, line_e, point), ERROR);

  // target point is on left side of the line
  point.x = 4.0;
  point.y = 8.0;
  ASSERT_NEAR(4.314555, calcLateralError2D(line_s, line_e, point), ERROR);

  // the length of line is zero
  ASSERT_NEAR(0.0, calcLateralError2D(line_s, line_s, point), ERROR);
}

TEST_F(LibWaypointFollowerTestSuite, calcRadius)
{
  geometry_msgs::Point target;
  geometry_msgs::Pose curr_pose;

  ASSERT_NEAR(1e9, calcRadius(target, curr_pose), ERROR);

  target.x = 10.0;
  target.y = 0.0;
  ASSERT_NEAR(1e9, calcRadius(target, curr_pose), ERROR);

  target.x = 10.0;
  target.y = 10.0;
  ASSERT_NEAR(10.0, calcRadius(target, curr_pose), ERROR);

  target.x = 0.0;
  target.y = 10.0;
  ASSERT_NEAR(5.0, calcRadius(target, curr_pose), ERROR);
}

TEST_F(LibWaypointFollowerTestSuite, extractPoses_WaypointArray)
{
  casper_auto_msgs::WaypointArray test;
  test.waypoints.emplace_back();
  test.waypoints.emplace_back();
  test.waypoints.emplace_back();
  test.waypoints.at(0).pose.pose.position.x = 5.0;
  test.waypoints.at(1).pose.pose.position.x = 100.0;
  test.waypoints.at(2).pose.pose.position.x = -999.0;
  std::vector<geometry_msgs::Pose> poses = extractPoses(test);
  ASSERT_NEAR(5.0, test.waypoints.at(0).pose.pose.position.x, ERROR);
  ASSERT_NEAR(100.0, test.waypoints.at(1).pose.pose.position.x, ERROR);
  ASSERT_NEAR(-999.0, test.waypoints.at(2).pose.pose.position.x, ERROR);
}
TEST_F(LibWaypointFollowerTestSuite, extractPoses_Waypoint)
{
  std::vector<autoware_msgs::Waypoint> test;
  test.emplace_back();
  test.emplace_back();
  test.emplace_back();
  test.at(0).pose.pose.position.x = 5.0;
  test.at(1).pose.pose.position.x = 100.0;
  test.at(2).pose.pose.position.x = -999.0;
  std::vector<geometry_msgs::Pose> poses = extractPoses(test);
  ASSERT_NEAR(5.0, test.at(0).pose.pose.position.x, ERROR);
  ASSERT_NEAR(100.0, test.at(1).pose.pose.position.x, ERROR);
  ASSERT_NEAR(-999.0, test.at(2).pose.pose.position.x, ERROR);
}

TEST_F(LibWaypointFollowerTestSuite, findClosestIdxWithDistAngThr)
{
  std::vector<geometry_msgs::Pose> curr_ps;
  geometry_msgs::Pose curr_pose;

  std::pair<bool, int32_t> ans;

  ans = findClosestIdxWithDistAngThr(curr_ps, curr_pose);
  ASSERT_EQ(false, ans.first);
  ASSERT_EQ(-1, ans.second);

  tf2::Quaternion tf2_q;
  tf2_q.setRPY(0, 0, 0);
  tf2_q.normalize();
  for (uint32_t i = 0; i < 10; i++)
  {
    geometry_msgs::Pose tmp_pose;
    tmp_pose.position.x = static_cast<double>(i) * 1.0;
    tmp_pose.orientation = tf2::toMsg(tf2_q);
    curr_ps.push_back(tmp_pose);
  }

  curr_pose.position.x = 0.0;
  curr_pose.orientation = tf2::toMsg(tf2_q);
  ans = findClosestIdxWithDistAngThr(curr_ps, curr_pose);
  ASSERT_EQ(true, ans.first);
  ASSERT_EQ(0, ans.second);

  tf2_q.setRPY(0, 0, 0.175);  // yaw = 10deg
  tf2_q.normalize();
  curr_pose.position.x = 3.0;
  curr_pose.orientation = tf2::toMsg(tf2_q);
  ans = findClosestIdxWithDistAngThr(curr_ps, curr_pose);
  ASSERT_EQ(true, ans.first);
  ASSERT_EQ(3, ans.second);
}

TEST_F(LibWaypointFollowerTestSuite, isDirectionForward)
{
  std::vector<geometry_msgs::Pose> poses;
  poses.reserve(2);
  geometry_msgs::Pose p0;
  geometry_msgs::Pose p1;
  geometry_msgs::Pose p2;

  // p1 and p2 is same pose
  poses.push_back(p0);
  poses.push_back(p1);
  poses.push_back(p2);
  ASSERT_EQ(false, isDirectionForward(poses));

  // p2.position.x = 3.0 p1 is origin position
  poses.clear();
  poses.push_back(p0);
  poses.push_back(p1);
  p2.position.x = 3.0;
  poses.push_back(p2);
  ASSERT_EQ(true, isDirectionForward(poses));

  // p2.position.x = -3.0 p1 is origin position
  poses.clear();
  poses.push_back(p0);
  poses.push_back(p1);
  p2.position.x = -3.0;
  poses.push_back(p2);
  ASSERT_EQ(false, isDirectionForward(poses));
}

TEST_F(LibWaypointFollowerTestSuite, normalizeEulerAngle)
{
  ASSERT_DOUBLE_EQ(M_PI, normalizeEulerAngle(3 * M_PI));
  ASSERT_DOUBLE_EQ(-M_PI, normalizeEulerAngle(-3 * M_PI));
  ASSERT_DOUBLE_EQ(-M_PI + 0.1, normalizeEulerAngle(M_PI + 0.1));
  ASSERT_DOUBLE_EQ(M_PI - 0.2, normalizeEulerAngle(-M_PI - 0.2));
}

TEST_F(LibWaypointFollowerTestSuite, transformToAbsoluteCoordinate2D)
{
  geometry_msgs::Point point;
  geometry_msgs::Pose origin;
  geometry_msgs::Point res;

  // no translation and rotation
  res = transformToAbsoluteCoordinate2D(point, origin);
  ASSERT_NEAR(0.0, res.x, ERROR);
  ASSERT_NEAR(0.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);

  // only translation
  point.x = 2.0;
  point.y = 2.0;
  origin.position.x = 5.0;
  origin.position.y = 3.0;

  res = transformToAbsoluteCoordinate2D(point, origin);
  ASSERT_NEAR(7.0, res.x, ERROR);
  ASSERT_NEAR(5.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);

  // translation and rotation 90 deg
  tf2::Quaternion tf_q;
  point.x = 3.0;
  point.y = 2.0;
  origin.position.x = 4.0;
  origin.position.y = 3.0;
  tf_q.setRPY(0.0, 0.0, 90 * M_PI / 180);
  origin.orientation = tf2::toMsg(tf_q);

  res = transformToAbsoluteCoordinate2D(point, origin);
  ASSERT_NEAR(2.0, res.x, ERROR);
  ASSERT_NEAR(6.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);

  // translation and rotation -90 deg
  point.x = 2.0;
  point.y = -3.0;
  origin.position.x = 5.0;
  origin.position.y = -4.0;
  tf_q.setRPY(0.0, 0.0, -90 * M_PI / 180);
  origin.orientation = tf2::toMsg(tf_q);

  res = transformToAbsoluteCoordinate2D(point, origin);
  ASSERT_NEAR(2.0, res.x, ERROR);
  ASSERT_NEAR(-6.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);
}

TEST_F(LibWaypointFollowerTestSuite, transformToRelativeCoordinate2D)
{
  geometry_msgs::Point point;
  geometry_msgs::Pose origin;
  geometry_msgs::Point res;

  // no translation and rotation
  res = transformToRelativeCoordinate2D(point, origin);
  ASSERT_NEAR(0.0, res.x, ERROR);
  ASSERT_NEAR(0.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);

  // only translation
  point.x = 7.0;
  point.y = 5.0;
  origin.position.x = 5.0;
  origin.position.y = 3.0;
  res = transformToRelativeCoordinate2D(point, origin);
  ASSERT_NEAR(2.0, res.x, ERROR);
  ASSERT_NEAR(2.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);

  // translation and rotation 90 deg
  tf2::Quaternion tf_q;

  point.x = 2.0;
  point.y = 6.0;
  origin.position.x = 4.0;
  origin.position.y = 3.0;
  tf_q.setRPY(0.0, 0.0, 90 * M_PI / 180);
  origin.orientation = tf2::toMsg(tf_q);
  res = transformToRelativeCoordinate2D(point, origin);
  ASSERT_NEAR(3.0, res.x, ERROR);
  ASSERT_NEAR(2.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);

  // translation and rotation -90 deg
  point.x = 2.0;
  point.y = -6.0;
  origin.position.x = 5.0;
  origin.position.y = -4.0;
  tf_q.setRPY(0.0, 0.0, -90 * M_PI / 180);
  origin.orientation = tf2::toMsg(tf_q);
  res = transformToRelativeCoordinate2D(point, origin);
  ASSERT_NEAR(2.0, res.x, ERROR);
  ASSERT_NEAR(-3.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "LibWaypointFollowerTestNode");
  ros::NodeHandle n;
  return RUN_ALL_TESTS();
}
