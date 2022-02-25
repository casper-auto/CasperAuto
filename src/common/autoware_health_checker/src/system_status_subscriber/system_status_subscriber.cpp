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

#include <autoware_health_checker/system_status_subscriber/system_status_subscriber.h>

namespace autoware_health_checker
{
SystemStatusSubscriber::SystemStatusSubscriber(
  ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh), pnh_(pnh) {}

void SystemStatusSubscriber::enable()
{
  status_sub_ = nh_.subscribe(
    "system_status", 10, &SystemStatusSubscriber::systemStatusCallback, this);
}

void SystemStatusSubscriber::systemStatusCallback(
  const autoware_system_msgs::SystemStatus::ConstPtr msg)
{
  for (auto& func : functions_)
  {
    auto pmsg = std::make_shared<autoware_system_msgs::SystemStatus>(*msg);
    func(pmsg);
  }
}

void SystemStatusSubscriber::addCallback(
  std::function<void(std::shared_ptr<autoware_system_msgs::SystemStatus>)> func)
{
  functions_.emplace_back(func);
}
}  // namespace autoware_health_checker
