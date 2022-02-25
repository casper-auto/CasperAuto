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

#ifndef AUTOWARE_HEALTH_CHECKER_HEALTH_AGGREGATOR_HEALTH_AGGREGATOR_H
#define AUTOWARE_HEALTH_CHECKER_HEALTH_AGGREGATOR_HEALTH_AGGREGATOR_H
// headers in ROS
#include <diagnostic_msgs/DiagnosticArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <ros/ros.h>

// headers in Autoware
#include <autoware_health_checker/constants.h>
#include <autoware_health_checker/health_checker/param_manager.h>
#include <autoware_health_checker/health_aggregator/status_monitor.h>
#include <autoware_system_msgs/NodeStatus.h>
#include <autoware_system_msgs/SystemStatus.h>

// headers in boost
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

// headers in STL
#include <map>
#include <mutex>
#include <string>
#include <vector>

class HealthAggregator
{
public:
  using ErrorLevel = autoware_health_checker::ErrorLevel;
  HealthAggregator(ros::NodeHandle nh, ros::NodeHandle pnh);
  void run();

private:
  using AwHwStatus = autoware_system_msgs::HardwareStatus;
  using AwHwStatusArray = std::vector<AwHwStatus>;
  using AwNodeStatus = autoware_system_msgs::NodeStatus;
  using AwDiagStatus = autoware_system_msgs::DiagnosticStatus;
  using AwDiagStatusArray = autoware_system_msgs::DiagnosticStatusArray;
  using AwSysStatus = autoware_system_msgs::SystemStatus;
  using RosDiagStatus = diagnostic_msgs::DiagnosticStatus;
  using RosDiagArr = diagnostic_msgs::DiagnosticArray;
  using ErrorKey = autoware_health_checker::ErrorKey;
  const ErrorLevel convertHardwareLevel(const ErrorLevel& level) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher system_status_pub_;
  std::map<ErrorLevel, ros::Publisher> text_pub_;
  ros::Subscriber node_status_sub_;
  ros::Subscriber diagnostic_array_sub_;
  ros::Timer system_status_timer_, vital_timer_, ros_observer_timer_;
  std::vector<std::string> detected_nodes_;
  StatusMonitor status_monitor_;
  void updateNodeStatus(const autoware_system_msgs::NodeStatus& node_status);
  void publishSystemStatus(const ros::TimerEvent& event);
  void nodeStatusCallback(const AwNodeStatus::ConstPtr& msg);
  void diagnosticArrayCallback(const RosDiagArr::ConstPtr& msg);
  std::string generateText(const std::vector<AwDiagStatus>& status);
  jsk_rviz_plugins::OverlayText
    generateOverlayText(const AwSysStatus& status, const ErrorLevel level);
  std::vector<AwDiagStatus>
    filterNodeStatus(const AwSysStatus& status, const ErrorLevel level);
  boost::optional<AwHwStatusArray> convert(const RosDiagArr::ConstPtr& msg);
  AwSysStatus system_status_;
  autoware_health_checker::ParamManager param_manager_;
  std::mutex mtx_;
  void updateConnectionStatus(const ros::TimerEvent& event);
  void rosObserverVitalCheck(const ros::TimerEvent& event);
  double hardware_diag_rate_;
  std::string hardware_diag_node_;
};
#endif  // AUTOWARE_HEALTH_CHECKER_HEALTH_AGGREGATOR_HEALTH_AGGREGATOR_H
