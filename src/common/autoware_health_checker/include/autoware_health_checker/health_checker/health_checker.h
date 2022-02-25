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

#ifndef AUTOWARE_HEALTH_CHECKER_HEALTH_CHECKER_HEALTH_CHECKER_H
#define AUTOWARE_HEALTH_CHECKER_HEALTH_CHECKER_HEALTH_CHECKER_H
// headers in ROS
#include <ros/ros.h>

// headers in Autoware
#include <autoware_health_checker/constants.h>
#include <autoware_health_checker/health_checker/diag_buffer.h>
#include <autoware_health_checker/health_checker/rate_checker.h>
#include <autoware_health_checker/health_checker/value_manager.h>
#include <autoware_system_msgs/NodeStatus.h>

// headers in STL
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <utility>
#include <thread>
#include <atomic>
#include <chrono>

// headers in boost
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace autoware_health_checker
{
using MinMax = std::pair<double, double>;
class HealthChecker
{
public:
  using AwDiagStatus = autoware_system_msgs::DiagnosticStatus;
  using AwDiagStatusArray = autoware_system_msgs::DiagnosticStatusArray;
  HealthChecker(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~HealthChecker();
  void ENABLE();
  ErrorLevel CHECK_MIN_VALUE(const ErrorKey& key, const double value,
    const double warn_value, const double error_value,
    const double fatal_value, const std::string& description);
  ErrorLevel CHECK_MAX_VALUE(const ErrorKey& key, const double value,
    const double warn_value, const double error_value,
    const double fatal_value, const std::string& description);
  ErrorLevel CHECK_RANGE(const ErrorKey& key, const double value,
    const MinMax warn_value, const MinMax error_value,
    const MinMax fatal_value, const std::string& description);
  template <class T> ErrorLevel CHECK_VALUE(
    const ErrorKey& key, const T& value,
    std::function<ErrorLevel(T value)> check_func,
    std::function<boost::property_tree::ptree(T value)> value_json_func,
    const std::string& description)
  {
    value_manager_.addCandidate(key);
    if (value_manager_.isNotFound(key))
    {
      return AwDiagStatus::UNDEFINED;
    }
    ErrorLevel check_result = check_func(value);
    boost::property_tree::ptree pt = value_json_func(value);
    std::stringstream ss;
    write_json(ss, pt);
    AwDiagStatus new_status;
    new_status.key = key;
    new_status.level = check_result;
    new_status.description = description;
    new_status.value = ss.str();
    new_status.header.stamp = ros::Time::now();
    new_status.type = AwDiagStatus::INVALID_VALUE;
    return SET_DIAG_STATUS(new_status);
  }
  void CHECK_RATE(const ErrorKey& key, const double warn_rate,
    const double error_rate, const double fatal_rate,
    const std::string& description);
  ErrorLevel CHECK_TRUE(const ErrorKey& key, const bool value,
    const ErrorLevel level, const std::string& description);
  ErrorLevel SET_DIAG_STATUS(
    const autoware_system_msgs::DiagnosticStatus& status);
  void NODE_ACTIVATE()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    node_activated_ = true;
  };
  void NODE_DEACTIVATE()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    node_activated_ = false;
  };
  bool getNodeStatus()
  {
    return node_activated_;
  };

private:
  std::vector<ErrorKey> getKeys();
  std::vector<ErrorKey> getRateCheckerKeys();
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::thread node_status_publish_thread_;
  ValueManager value_manager_;
  std::map<ErrorKey, std::unique_ptr<DiagBuffer>> diag_buffers_;
  std::map<ErrorKey, std::unique_ptr<RateChecker>> rate_checkers_;
  ros::Publisher status_pub_;
  bool keyExist(const ErrorKey& key) const;
  bool addNewBuffer(const ErrorKey& key, const ErrorType type,
    const std::string& description);
  template <typename T> autoware_system_msgs::DiagnosticStatus
    setValueCommon(
      const ErrorKey& key, const T& value, const std::string& desc);
  template <typename T> std::string valueToJson(const T& value)
  {
    std::stringstream ss;
    boost::property_tree::ptree pt;
    pt.put("value", value);
    write_json(ss, pt);
    return ss.str();
  }
  void publishStatus();
  bool node_activated_;
  std::atomic<bool> is_shutdown_;
  std::mutex mtx_;
};
}  // namespace autoware_health_checker
#endif  // AUTOWARE_HEALTH_CHECKER_HEALTH_CHECKER_HEALTH_CHECKER_H
