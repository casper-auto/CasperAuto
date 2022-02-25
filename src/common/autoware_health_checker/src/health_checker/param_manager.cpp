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

#include <set>
#include <autoware_health_checker/health_checker/param_manager.h>

namespace autoware_health_checker
{

ParamManager::ParamManager(ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh), pnh_(pnh)
{
  nh_.getParam("health_checker", params_);
  get_timer_ =
    nh_.createTimer(ros::Duration(1.0), &ParamManager::getParams, this);
  set_timer_ =
    nh_.createTimer(ros::Duration(1.0), &ParamManager::setParams, this);
}

void ParamManager::addCandidate(const ErrorKey& key)
{
  std::lock_guard<std::mutex> lock(key_mtx_);
  cached_candidate_key_.emplace(key);
}

bool ParamManager::isNotFound(const ErrorKey& key)
{
  std::lock_guard<std::mutex> lock(param_mtx_);
  return !params_.valid() || !params_.hasMember(key);
}

bool ParamManager::isNotFound(const ErrorKey& ns, const ErrorKey& key)
{
  std::lock_guard<std::mutex> lock(param_mtx_);
  return !params_.valid() || !params_.hasMember(ns)
    ? true : !params_[ns].hasMember(key);
}

const XmlRpc::XmlRpcValue& ParamManager::getParams()
{
  std::lock_guard<std::mutex> lock(param_mtx_);
  return params_;
}

void ParamManager::getParams(const ros::TimerEvent& event)
{
  XmlRpc::XmlRpcValue param;
  nh_.getParam("health_checker", param);
  // Even if it is delayed at the time of getParam(),
  // it is necessary to prevent the delay from affecting getValue().
  std::lock_guard<std::mutex> lock(param_mtx_);
  params_ = param;
}

void ParamManager::setParams(const ros::TimerEvent& event)
{
  std::set<ErrorKey> keys;
  {
    std::lock_guard<std::mutex> lock(key_mtx_);
    keys = cached_candidate_key_;
    cached_candidate_key_.clear();
  }
  static std::set<ErrorKey> previous_keys;
  for (const auto& key : keys)
  {
    if (previous_keys.count(key) == 0)
    {
      nh_.setParam("diag_reference/" + key, "default");
    }
  }
  previous_keys.insert(keys.begin(), keys.end());
}
}  // namespace autoware_health_checker
