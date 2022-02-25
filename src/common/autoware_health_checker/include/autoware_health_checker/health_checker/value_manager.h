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

#ifndef AUTOWARE_HEALTH_CHECKER_HEALTH_CHECKER_VALUE_MANAGER_H
#define AUTOWARE_HEALTH_CHECKER_HEALTH_CHECKER_VALUE_MANAGER_H

// headers in ROS
#include <ros/ros.h>

// headers in Autoware
#include <autoware_health_checker/constants.h>
#include <autoware_health_checker/health_checker/param_manager.h>

// headers in STL
#include <map>
#include <mutex>
#include <string>
#include <utility>

// headers in Boost
#include <boost/optional.hpp>
namespace autoware_health_checker
{
class ValueManager : public ParamManager
{
public:
  using ErrorCategory = std::pair<ErrorKey, ThreshType>;
  using ErrorSummary = std::pair<ErrorCategory, ErrorLevel>;

  ValueManager(ros::NodeHandle nh, ros::NodeHandle pnh);
  void setDefaultValue(
    const ErrorKey& key, const ThreshType& thresh_type, const double warn_value,
    const double error_value, const double fatal_value);
  boost::optional<double> getValue(const ErrorKey& key,
    const ThreshType& thresh_type, const ErrorLevel level);
  bool isEmpty(const ErrorKey& key) const;

private:
  using AwDiagStatus = autoware_system_msgs::DiagnosticStatus;
  std::map<ErrorSummary, double> error_details_;
};
}  // namespace autoware_health_checker
#endif  // AUTOWARE_HEALTH_CHECKER_HEALTH_CHECKER_VALUE_MANAGER_H
