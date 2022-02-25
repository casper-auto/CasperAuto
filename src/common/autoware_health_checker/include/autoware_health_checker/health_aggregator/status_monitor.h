/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
#ifndef AUTOWARE_HEALTH_CHECKER_HEALTH_AGGREGATOR_STATUS_MONITOR_H
#define AUTOWARE_HEALTH_CHECKER_HEALTH_AGGREGATOR_STATUS_MONITOR_H

#include <string>
#include <map>
#include <mutex>
#include <ros/ros.h>
#include <autoware_health_checker/constants.h>
#include <autoware_system_msgs/NodeStatus.h>

class TimeoutManager
{
public:
  TimeoutManager() = default;
  explicit TimeoutManager(const double time_out)
    : timer_start_time_(ros::Time::now()), time_out_(time_out) {}
  double getDuration(const ros::Time& current) const
  {
    return (current - timer_start_time_).toSec();
  }
  bool isOverLimit(const ros::Time& current) const
  {
    return (getDuration(current) > time_out_);
  }
  const ros::Time& getStartTime() const
  {
    return timer_start_time_;
  }
private:
  ros::Time timer_start_time_;
  double time_out_;
};

class StatusMonitor
{
public:
  StatusMonitor();
  void updateStamp(const std::string& name, const double timeout);
  autoware_system_msgs::NodeStatus getMonitorStatus() const;
private:
  std::mutex mtx_;
  std::map<std::string, TimeoutManager> timeout_manager_array_;
};

#endif  // AUTOWARE_HEALTH_CHECKER_HEALTH_AGGREGATOR_STATUS_MONITOR_H
