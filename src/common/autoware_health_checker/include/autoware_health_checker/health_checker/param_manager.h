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

#ifndef AUTOWARE_HEALTH_CHECKER_HEALTH_CHECKER_PARAM_MANAGER_H
#define AUTOWARE_HEALTH_CHECKER_HEALTH_CHECKER_PARAM_MANAGER_H

#include <string>
#include <set>
#include <mutex>
#include <ros/ros.h>
#include <autoware_health_checker/constants.h>

namespace autoware_health_checker
{
class ParamManager
{
public:
  ParamManager(ros::NodeHandle nh, ros::NodeHandle pnh);
  bool isNotFound(const ErrorKey& key);
  bool isNotFound(const ErrorKey& ns, const ErrorKey& key);
  void addCandidate(const ErrorKey& key);
  const XmlRpc::XmlRpcValue& getParams();
private:
  void getParams(const ros::TimerEvent& event);
  void setParams(const ros::TimerEvent& event);
  ros::NodeHandle nh_, pnh_;
  ros::Timer get_timer_, set_timer_;
  std::set<ErrorKey> cached_candidate_key_;
  XmlRpc::XmlRpcValue params_;
  std::mutex param_mtx_, key_mtx_;
};
}  // namespace autoware_health_checker
#endif  // AUTOWARE_HEALTH_CHECKER_HEALTH_CHECKER_PARAM_MANAGER_H
