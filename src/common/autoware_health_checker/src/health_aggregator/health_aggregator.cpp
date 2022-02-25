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
#include <vector>
#include <regex>
#include <autoware_health_checker/health_aggregator/health_aggregator.h>
#include <ros_observer/lib_ros_observer.h>

namespace
{
std::string changeToKeyFormat(const std::string& node_name)
{
  std::string changed_name(node_name);
  if (changed_name.at(0) == '/')
  {
    changed_name.erase(changed_name.begin());
  }
  changed_name = std::regex_replace(changed_name, std::regex("/"), "_");
  return changed_name;
}


boost::optional <std::string> getValidName(const std::string& orig)
{
  std::string changed(orig);
  static const std::vector<std::string> delete_strings =
  {
    ".*:", ".*/", R"(\(.*\))",
    R"([\[\]"\\(){}?.*+^$|!#%&'-=~`@;:,<> ])"
  };
  for (const auto& str : delete_strings)
  {
    changed = std::regex_replace(changed, std::regex(str), "");
  }
  std::string error;
  return ros::names::validate(changed, error) ?
    boost::optional<std::string>(changed) : boost::none;
}
}  // namespace

HealthAggregator::HealthAggregator(ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh), pnh_(pnh), param_manager_(nh_, pnh)
{
  nh_.param("hardware_diag_node",
    hardware_diag_node_, std::string("diagnostic_aggregator"));
  nh_.param(hardware_diag_node_ + "/pub_rate", hardware_diag_rate_, 1.0);
}

void HealthAggregator::run()
{
  system_status_pub_ = nh_.advertise<AwSysStatus>("system_status", 10);
  auto registerTextPublisher = [this](ErrorLevel level, std::string topic)
  {
    text_pub_[level] = pnh_.advertise<jsk_rviz_plugins::OverlayText>(topic, 1);
    return;
  };
  registerTextPublisher(AwDiagStatus::OK, "ok_text");
  registerTextPublisher(AwDiagStatus::WARN, "warn_text");
  registerTextPublisher(AwDiagStatus::ERROR, "error_text");
  registerTextPublisher(AwDiagStatus::FATAL, "fatal_text");
  node_status_sub_ = nh_.subscribe("node_status", 10,
    &HealthAggregator::nodeStatusCallback, this);
  // ros::master::getNodes will continue to wait for a response from the master
  // if the master goes down unless a timeout is specified.
  // To avoid this, it is necessary to set a timeout.
  ros::master::setRetryTimeout(ros::WallDuration(0.01));
  diagnostic_array_sub_ = nh_.subscribe(
    "diagnostics_agg", 10, &HealthAggregator::diagnosticArrayCallback, this);

  ros::Duration duration(1.0 / autoware_health_checker::SYSTEM_UPDATE_RATE);
  system_status_timer_ =
    nh_.createTimer(duration, &HealthAggregator::publishSystemStatus, this);
  vital_timer_ =
    nh_.createTimer(duration, &HealthAggregator::updateConnectionStatus, this);
  ros_observer_timer_ =
    nh_.createTimer(duration, &HealthAggregator::rosObserverVitalCheck, this);
}

void HealthAggregator::updateNodeStatus(
  const autoware_system_msgs::NodeStatus& node_status)
{
  auto& node_status_array = system_status_.node_status;
  auto identify = [&node_status](const autoware_system_msgs::NodeStatus& status)
  {
    return status.node_name == node_status.node_name;
  };
  auto result =
    std::find_if(node_status_array.begin(), node_status_array.end(), identify);
  if (result != node_status_array.end())
  {
    *result = node_status;
  }
  else
  {
    node_status_array.emplace_back(node_status);
  }
}
void HealthAggregator::publishSystemStatus(const ros::TimerEvent& event)
{
  std::lock_guard<std::mutex> lock(mtx_);
  system_status_.header.stamp = ros::Time::now();
  updateNodeStatus(status_monitor_.getMonitorStatus());
  system_status_.available_nodes = detected_nodes_;
  system_status_pub_.publish(system_status_);
  static const std::array<ErrorLevel, 4> level_array =
  {
    AwDiagStatus::OK,
    AwDiagStatus::WARN,
    AwDiagStatus::ERROR,
    AwDiagStatus::FATAL
  };
  for (const auto& level : level_array)
  {
    text_pub_[level].publish(generateOverlayText(system_status_, level));
  }
}

void HealthAggregator::rosObserverVitalCheck(const ros::TimerEvent& event)
{
  static constexpr double loop_rate = autoware_health_checker::SYSTEM_UPDATE_RATE;
  static ShmVitalMonitor shm_ROvmon("RosObserver", loop_rate, VitalMonitorMode::CNT_MON);
  static ShmVitalMonitor shm_HAvmon("HealthAggregator", loop_rate);

  shm_ROvmon.run();
  shm_HAvmon.run();
}

void HealthAggregator::updateConnectionStatus(const ros::TimerEvent& event)
{
  std::vector<std::string> detected_nodes;
  ros::master::getNodes(detected_nodes);
  detected_nodes_ = detected_nodes;
}

void HealthAggregator::nodeStatusCallback(const AwNodeStatus::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  updateNodeStatus(*msg);
  static const double timeout =
    1.0 / autoware_health_checker::NODE_STATUS_UPDATE_RATE * 2.0;
  status_monitor_.updateStamp(changeToKeyFormat(msg->node_name), timeout);
}

void HealthAggregator::diagnosticArrayCallback(const RosDiagArr::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto status = convert(msg);
  if (status)
  {
    system_status_.hardware_status = status.get();
  }
  static const double timeout = 1.0 / hardware_diag_rate_ * 2.0;
  status_monitor_.updateStamp(changeToKeyFormat(hardware_diag_node_), timeout);
}

std::string HealthAggregator::generateText(
  const std::vector<AwDiagStatus>& status)
{
  std::string text;
  for (const auto& s : status)
  {
    text = text + s.description + "\n";
  }
  return text;
}

jsk_rviz_plugins::OverlayText
HealthAggregator::generateOverlayText(const AwSysStatus& status,
  const HealthAggregator::ErrorLevel level)
{
  jsk_rviz_plugins::OverlayText text;
  text.action = text.ADD;
  text.width = 640;
  text.height = 640;
  text.top = 0;
  text.bg_color.r = 0;
  text.bg_color.g = 0;
  text.bg_color.b = 0;
  text.bg_color.a = 0.7;
  text.text_size = 20.0;
  if (level == AwDiagStatus::OK)
  {
    text.left = 0;
    text.fg_color.r = 0.0;
    text.fg_color.g = 0.0;
    text.fg_color.b = 1.0;
    text.fg_color.a = 1.0;
    text.text = generateText(filterNodeStatus(status, level));
  }
  else if (level == AwDiagStatus::WARN)
  {
    text.left = 640 * 1;
    text.fg_color.r = 1.0;
    text.fg_color.g = 1.0;
    text.fg_color.b = 0.0;
    text.fg_color.a = 1.0;
    text.text = generateText(filterNodeStatus(status, level));
  }
  else if (level == AwDiagStatus::ERROR)
  {
    text.left = 640 * 2;
    text.fg_color.r = 1.0;
    text.fg_color.g = 0.0;
    text.fg_color.b = 0.0;
    text.fg_color.a = 1.0;
    text.text = generateText(filterNodeStatus(status, level));
  }
  else if (level == AwDiagStatus::FATAL)
  {
    text.left = 640 * 3;
    text.fg_color.r = 1.0;
    text.fg_color.g = 1.0;
    text.fg_color.b = 1.0;
    text.fg_color.a = 1.0;
    text.text = generateText(filterNodeStatus(status, level));
  }
  return text;
}

std::vector<HealthAggregator::AwDiagStatus>
HealthAggregator::filterNodeStatus(const AwSysStatus& status,
  const HealthAggregator::ErrorLevel level)
{
  std::vector<AwDiagStatus> ret;
  for (const auto& node_status : status.node_status)
  {
    if (!node_status.node_activated)
    {
      continue;
    }
    for (const auto& node_status_array : node_status.status)
    {
      for (const auto& diag_status : node_status_array.status)
      {
        if (diag_status.level == level)
        {
          ret.emplace_back(diag_status);
        }
      }
    }
  }
  return ret;
}

const HealthAggregator::ErrorLevel
  HealthAggregator::convertHardwareLevel(const ErrorLevel& level) const
{
  return
    (level == RosDiagStatus::OK) ? AwDiagStatus::OK :
    (level == RosDiagStatus::WARN) ? AwDiagStatus::WARN :
    (level == RosDiagStatus::ERROR) ? AwDiagStatus::ERROR :
    (level == RosDiagStatus::STALE) ? AwDiagStatus::FATAL :
    AwDiagStatus::UNDEFINED;
}

boost::optional<HealthAggregator::AwHwStatusArray>
  HealthAggregator::convert(const RosDiagArr::ConstPtr& msg)
{
  AwHwStatusArray status_array;
  if (msg->status.empty())
  {
    return boost::none;
  }

  for (const auto& hw_status : msg->status)
  {
    const auto ns = getValidName(hw_status.name);
    if (!ns)
    {
      continue;
    }
    AwHwStatus status;
    status.header = msg->header;
    status.hardware_name = ns.get();
    const ErrorLevel level = convertHardwareLevel(hw_status.level);
    AwDiagStatusArray diag_array;
    for (const auto& diag_status : hw_status.values)
    {
      const auto local_key = getValidName(diag_status.key);
      if (!local_key)
      {
        continue;
      }
      const ErrorKey global_key = ns.get() + "/" + local_key.get();
      param_manager_.addCandidate(global_key);
      if (param_manager_.isNotFound(ns.get(), local_key.get()))
      {
        continue;
      }
      AwDiagStatus diag;
      diag.header = msg->header;
      diag.key = global_key;
      diag.description = global_key;
      diag.type = AwDiagStatus::HARDWARE;
      diag.level = level;
      std::stringstream ss;
      boost::property_tree::ptree pt;
      pt.put("value", diag_status.value);
      write_json(ss, pt);
      diag.value = ss.str();
      diag_array.status.emplace_back(diag);
    }
    if (!diag_array.status.empty())
    {
      status.status.emplace_back(diag_array);
      status_array.emplace_back(status);
    }
  }
  return status_array;
}
