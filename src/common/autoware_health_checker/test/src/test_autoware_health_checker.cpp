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
 *
 * v1.0 Masaya Kataoka
 */
#include <autoware_health_checker/health_checker/health_checker.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include <utility>

using AwDiagStatus = autoware_system_msgs::DiagnosticStatus;
using ErrorLevel = autoware_health_checker::ErrorLevel;
template <class T>
using InputAndResult = std::vector<std::pair<T, ErrorLevel>>;

class AutowareHealthCheckerTestClass
{
public:
  AutowareHealthCheckerTestClass() : pnh("~") {}
  std::shared_ptr<autoware_health_checker::HealthChecker> health_checker_ptr;
  ros::NodeHandle pnh;
  ros::NodeHandle nh;
  void init()
  {
    ros::param::set("health_checker/test", "default");
    health_checker_ptr =
      std::make_shared<autoware_health_checker::HealthChecker>(nh, pnh);
  }
};

class AutowareHealthCheckerTestSuite : public ::testing::Test
{
public:
  AutowareHealthCheckerTestSuite() {}

  ~AutowareHealthCheckerTestSuite() {}
  AutowareHealthCheckerTestClass test_obj_;

protected:
  virtual void SetUp()
  {
    test_obj_.init();
  }
  virtual void TearDown() {}
};

autoware_health_checker::ErrorLevel test_function(double value)
{
  return
    (value == 0.0) ? AwDiagStatus::FATAL :
    (value == 1.0) ? AwDiagStatus::ERROR :
    (value == 2.0) ? AwDiagStatus::WARN : AwDiagStatus::OK;
};

boost::property_tree::ptree test_value_json_func(double value)
{
  boost::property_tree::ptree tree;
  tree.put("value", value);
  return tree;
};

/*
  test for value check function
*/
TEST_F(AutowareHealthCheckerTestSuite, CHECK_VALUE)
{
  using CheckLevel = std::function<ErrorLevel(double value)>;
  using CheckVal = std::function<boost::property_tree::ptree(double value)>;
  CheckLevel check_func = test_function;
  CheckVal check_value_json_func = test_value_json_func;

  const InputAndResult<double> dataset =
  {
    std::make_pair(0.0, AwDiagStatus::FATAL),
    std::make_pair(1.0, AwDiagStatus::ERROR),
    std::make_pair(2.0, AwDiagStatus::WARN),
    std::make_pair(-1.0, AwDiagStatus::OK)
  };
  for (const auto& data : dataset)
  {
    auto ret_value = test_obj_.health_checker_ptr->CHECK_VALUE(
      "test", data.first, check_func, check_value_json_func, "test");
    ASSERT_EQ(ret_value, data.second)
      << "CHECK_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(data.second);
  }
  auto value = check_value_json_func(0.0).get_optional<double>("value");
  ASSERT_EQ(value.get(), 0.0)
    << "Failed to get json value."
    << "It should be 0.0";
}

/*
  test for minimum value check function
*/
TEST_F(AutowareHealthCheckerTestSuite, CHECK_MIN_VALUE)
{
  const InputAndResult<double> dataset =
  {
    std::make_pair(1.0, AwDiagStatus::FATAL),
    std::make_pair(3.0, AwDiagStatus::ERROR),
    std::make_pair(5.0, AwDiagStatus::WARN),
    std::make_pair(7.0, AwDiagStatus::OK)
  };
  for (const auto& data : dataset)
  {
    auto ret_min = test_obj_.health_checker_ptr->CHECK_MIN_VALUE(
      "test", data.first, 6, 4, 2, "test");
    ASSERT_EQ(ret_min, data.second)
      << "CHECK_MIN_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(data.second);
  }
}

/*
  test for maximum value check function
*/
TEST_F(AutowareHealthCheckerTestSuite, CHECK_MAX_VALUE)
{
  const InputAndResult<double> dataset =
  {
    std::make_pair(7.0, AwDiagStatus::FATAL),
    std::make_pair(5.0, AwDiagStatus::ERROR),
    std::make_pair(3.0, AwDiagStatus::WARN),
    std::make_pair(1.0, AwDiagStatus::OK)
  };
  for (const auto& data : dataset)
  {
    auto ret_max = test_obj_.health_checker_ptr->CHECK_MAX_VALUE(
      "test", data.first, 2, 4, 6, "test");
    ASSERT_EQ(ret_max, data.second)
      << "CHECK_MAX_VALUE function returns invalid value."
      << "It should be " << static_cast<int>(data.second);
  }
}

/*
  test for range check functions
*/
TEST_F(AutowareHealthCheckerTestSuite, CHECK_RANGE)
{
  const std::pair<double, double> warn_range = std::make_pair(2.0, 4.0);
  const std::pair<double, double> error_range = std::make_pair(1.0, 5.0);
  const std::pair<double, double> fatal_range = std::make_pair(0.0, 6.0);
  const InputAndResult<double> dataset =
  {
    std::make_pair(7.0, AwDiagStatus::FATAL),
    std::make_pair(5.5, AwDiagStatus::ERROR),
    std::make_pair(4.5, AwDiagStatus::WARN),
    std::make_pair(3.0, AwDiagStatus::OK)
  };

  for (const auto& data : dataset)
  {
    auto ret_range = test_obj_.health_checker_ptr->CHECK_RANGE(
      "test", data.first, warn_range, error_range, fatal_range, "test");
    ASSERT_EQ(ret_range, data.second)
      << "CHECK_RANGE function returns invalid value."
      << "It should be " << static_cast<int>(data.second);
  }
}

/*
  test for bool check functions
*/
TEST_F(AutowareHealthCheckerTestSuite, CHECK_TRUE)
{
  const InputAndResult<bool> dataset =
  {
    std::make_pair(true, AwDiagStatus::FATAL),
    std::make_pair(false, AwDiagStatus::ERROR),
    std::make_pair(true, AwDiagStatus::WARN),
    std::make_pair(false, AwDiagStatus::OK)
  };
  for (const auto& data : dataset)
  {
    auto ret_bool = test_obj_.health_checker_ptr->CHECK_TRUE(
      "test", data.first, data.second, "test");
    ASSERT_EQ(ret_bool, data.second)
      << "CHECK_TRUE function returns invalid value."
      << "It should be " << static_cast<int>(data.second);
  }
}

/*
  test for set diag function
*/
TEST_F(AutowareHealthCheckerTestSuite, SET_DIAG_STATUS)
{
  const InputAndResult<ErrorLevel> dataset =
  {
    std::make_pair(AwDiagStatus::FATAL, AwDiagStatus::FATAL),
    std::make_pair(AwDiagStatus::ERROR, AwDiagStatus::ERROR),
    std::make_pair(AwDiagStatus::WARN, AwDiagStatus::WARN),
    std::make_pair(AwDiagStatus::OK, AwDiagStatus::OK)
  };
  autoware_system_msgs::DiagnosticStatus status;
  status.key = "test";
  for (const auto& data : dataset)
  {
    status.level = data.first;
    auto ret_diag = test_obj_.health_checker_ptr->SET_DIAG_STATUS(status);
    ASSERT_EQ(ret_diag, data.second)
      << "SET_DIAG_STATUS function returns invalid value."
      << "It should be " << static_cast<int>(data.second);
  }
}

/*
  test for node status
*/
TEST_F(AutowareHealthCheckerTestSuite, NODE_STATUS)
{
  test_obj_.health_checker_ptr->NODE_ACTIVATE();
  auto ret_active = test_obj_.health_checker_ptr->getNodeStatus();
  ASSERT_EQ(ret_active, true) << "The value must be true";
  test_obj_.health_checker_ptr->NODE_DEACTIVATE();
  auto ret_inactive = test_obj_.health_checker_ptr->getNodeStatus();
  ASSERT_EQ(ret_inactive, false) << "The value must be true";
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "AutowareHealthCheckerTestNode");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
