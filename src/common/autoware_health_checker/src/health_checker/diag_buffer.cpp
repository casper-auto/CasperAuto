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
#include <algorithm>
#include <autoware_health_checker/health_checker/diag_buffer.h>

namespace autoware_health_checker
{

DiagBuffer::DiagBuffer(ErrorKey key, ErrorType type,
  std::string description, double buffer_duration)
  : type(type)
  , description(description)
  , key_(key)
  , buffer_duration_(ros::Duration(buffer_duration)) {}

void DiagBuffer::addDiag(autoware_system_msgs::DiagnosticStatus status)
{
  std::lock_guard<std::mutex> lock(mtx_);
  buffer_[status.level].status.emplace_back(status);
  updateBuffer();
}

autoware_system_msgs::DiagnosticStatusArray DiagBuffer::getAndClearData()
{
  static const std::array<ErrorLevel, 4> level_array =
  {
    AwDiagStatus::ERROR,
    AwDiagStatus::WARN,
    AwDiagStatus::OK,
    AwDiagStatus::UNDEFINED
  };
  std::lock_guard<std::mutex> lock(mtx_);
  updateBuffer();
  auto data = buffer_.at(AwDiagStatus::FATAL);
  auto& status = data.status;
  for (const auto& level : level_array)
  {
    const auto& buf_status = buffer_.at(level).status;
    status.insert(status.end(), buf_status.begin(), buf_status.end());
  }
  std::sort(status.begin(), status.end(),
    std::bind(&DiagBuffer::isOlderTimestamp, this,
    std::placeholders::_1, std::placeholders::_2));
  buffer_.clear();
  return data;
}

ErrorLevel DiagBuffer::getErrorLevel()
{
  static const std::array<ErrorLevel, 4> level_array =
  {
    AwDiagStatus::FATAL,
    AwDiagStatus::ERROR,
    AwDiagStatus::WARN,
    AwDiagStatus::OK
  };
  std::lock_guard<std::mutex> lock(mtx_);
  updateBuffer();
  for (const auto& level : level_array)
  {
    if (!buffer_.at(level).status.empty())
    {
      return level;
    }
  }
  return AwDiagStatus::OK;
}

// filter data from timestamp and level
autoware_system_msgs::DiagnosticStatusArray
DiagBuffer::filterBuffer(ros::Time now, ErrorLevel level)
{
  autoware_system_msgs::DiagnosticStatusArray filtered_data;
  if (buffer_.count(level) != 0)
  {
    filtered_data = buffer_.at(level);
  }
  autoware_system_msgs::DiagnosticStatusArray ret;
  for (const auto& data : filtered_data.status)
  {
    if ((data.header.stamp + buffer_duration_) > now)
    {
      ret.status.emplace_back(data);
    }
  }
  return ret;
}

void DiagBuffer::updateBuffer()
{
  static const std::array<ErrorLevel, 5> level_array =
  {
    AwDiagStatus::FATAL,
    AwDiagStatus::ERROR,
    AwDiagStatus::WARN,
    AwDiagStatus::OK,
    AwDiagStatus::UNDEFINED
  };
  ros::Time now = ros::Time::now();
  for (const auto& level : level_array)
  {
    buffer_[level] = filterBuffer(now, level);
  }
}

bool DiagBuffer::isOlderTimestamp(
    const autoware_system_msgs::DiagnosticStatus &a,
    const autoware_system_msgs::DiagnosticStatus &b)
{
  return a.header.stamp < b.header.stamp;
}
}  // namespace autoware_health_checker
