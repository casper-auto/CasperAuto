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

#ifndef AUTOWARE_HEALTH_CHECKER_HEALTH_CHECKER_RATE_CHECKER_H
#define AUTOWARE_HEALTH_CHECKER_HEALTH_CHECKER_RATE_CHECKER_H
// headers in ROS
#include <ros/ros.h>

// headers in STL
#include <mutex>
#include <vector>
#include <string>
#include <utility>

// headers in Boost
#include <boost/optional.hpp>

// headers in Autoware
#include <autoware_health_checker/constants.h>

namespace autoware_health_checker
{
using LevelRatePair = std::pair<ErrorLevel, double>;

class RateChecker
{
public:
  RateChecker(double buffer_duration, double warn_rate, double error_rate,
              double fatal_rate, std::string description);
  void check();
  boost::optional<LevelRatePair> getErrorLevelAndRate();
  boost::optional<ErrorLevel> getErrorLevel();
  boost::optional<double> getRate();
  void setRate(double warn_rate, double error_rate, double fatal_rate);
  const std::string description;

private:
  using AwDiagStatus = autoware_system_msgs::DiagnosticStatus;
  ros::Time start_time_;
  void update();
  std::vector<ros::Time> data_;
  double buffer_duration_;
  double warn_rate_;
  double error_rate_;
  double fatal_rate_;
  std::mutex mtx_;
};
}  // namespace autoware_health_checker
#endif  // AUTOWARE_HEALTH_CHECKER_HEALTH_CHECKER_RATE_CHECKER_H
