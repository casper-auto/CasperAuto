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

#include <string>
#include <autoware_health_checker/health_checker/value_manager.h>
namespace autoware_health_checker
{
ValueManager::ValueManager(ros::NodeHandle nh, ros::NodeHandle pnh)
  : ParamManager(nh, pnh) {}

void ValueManager::setDefaultValue(
  const ErrorKey& key, const ThreshType& thresh_type,
  const double warn_value, const double error_value, const double fatal_value)
{
  const auto category = std::make_pair(key, thresh_type);
  error_details_[std::make_pair(category, AwDiagStatus::WARN)] = warn_value;
  error_details_[std::make_pair(category, AwDiagStatus::ERROR)] = error_value;
  error_details_[std::make_pair(category, AwDiagStatus::FATAL)] = fatal_value;
}

boost::optional<double> ValueManager::getValue(const ErrorKey& key,
  const ThreshType& thresh_type, const ErrorLevel level)
{
  const std::string level_name =
    (level == AwDiagStatus::WARN) ? "warn" :
    (level == AwDiagStatus::ERROR) ? "error" :
    (level == AwDiagStatus::FATAL) ? "fatal" :
    std::string();
  auto diag_params = getParams();
  if (level_name.empty() || !diag_params.hasMember(key))
  {
    return boost::none;
  }
  const bool has_level = diag_params[key].hasMember(thresh_type) ?
    diag_params[key][thresh_type].hasMember(level_name) : false;
  const auto category = std::make_pair(key, thresh_type);
  return (has_level) ?
    static_cast<double>(diag_params[key][thresh_type][level_name]) :
    error_details_.at(std::make_pair(category, level));
}
}  // namespace autoware_health_checker
