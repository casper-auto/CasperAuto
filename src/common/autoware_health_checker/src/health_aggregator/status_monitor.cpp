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
#include <string>
#include <autoware_health_checker/health_aggregator/status_monitor.h>

StatusMonitor::StatusMonitor() {}

void StatusMonitor::updateStamp(const std::string& name, const double timeout)
{
  if (name.empty())
  {
    return;
  }
  std::lock_guard<std::mutex> lock(mtx_);
  timeout_manager_array_[name] = TimeoutManager(timeout);
}

autoware_system_msgs::NodeStatus StatusMonitor::getMonitorStatus() const
{
  using DiagStatus = autoware_system_msgs::DiagnosticStatus;
  using DiagStatusArray = autoware_system_msgs::DiagnosticStatusArray;
  static const std::string my_name = ros::this_node::getName();
  const ros::Time current = ros::Time::now();
  autoware_system_msgs::NodeStatus my_status;
  DiagStatus status_template;
  status_template.key = "node_status_rate_slow";
  status_template.description = "node_status rate slow";
  status_template.type = DiagStatus::UNEXPECTED_RATE;
  status_template.level = DiagStatus::OK;
  for (const auto& timeout_manager : timeout_manager_array_)
  {
    DiagStatusArray status_array;
    DiagStatus status(status_template);
    const auto& node_name = timeout_manager.first;
    const auto& node_timer = timeout_manager.second;
    status.key = node_name + "_" + status.key;
    status.description = node_name + " " + status.description;
    std::stringstream ss;
    ss << node_timer.getDuration(current);
    status.value = ss.str();
    if (node_timer.isOverLimit(current))
    {
      status.level = DiagStatus::ERROR;
    }
    status.header.stamp = node_timer.getStartTime();
    status_array.status.emplace_back(status);
    my_status.status.emplace_back(status_array);
  }
  my_status.header.stamp = ros::Time::now();
  my_status.node_name = my_name;
  my_status.node_activated = true;
  return my_status;
}
