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
#include <autoware_health_checker/health_checker/rate_checker.h>

namespace autoware_health_checker
{
RateChecker::RateChecker(
  double buffer_duration, double warn_rate, double error_rate,
  double fatal_rate, std::string description)
  : buffer_duration_(buffer_duration), warn_rate_(warn_rate),
    error_rate_(error_rate), fatal_rate_(fatal_rate),
    description(description), start_time_(ros::Time::now()) {}

boost::optional<LevelRatePair> RateChecker::getErrorLevelAndRate()
{
  const auto rate = getRate();
  if (!rate)
  {
    return boost::none;
  }
  const ErrorLevel level =
    (rate.get() < fatal_rate_) ? AwDiagStatus::FATAL :
    (rate.get() < error_rate_) ? AwDiagStatus::ERROR :
    (rate.get() < warn_rate_) ? AwDiagStatus::WARN :
    AwDiagStatus::OK;
  return std::make_pair(level, rate.get());
}

boost::optional<ErrorLevel> RateChecker::getErrorLevel()
{
  auto level_and_rate = getErrorLevelAndRate();
  if (!level_and_rate)
  {
    return boost::none;
  }
  return level_and_rate.get().first;
}

void RateChecker::setRate(
  double warn_rate, double error_rate, double fatal_rate)
{
  update();
  std::lock_guard<std::mutex> lock(mtx_);
  warn_rate_ = warn_rate;
  error_rate_ = error_rate;
  fatal_rate_ = fatal_rate;
}

void RateChecker::check()
{
  update();
  std::lock_guard<std::mutex> lock(mtx_);
  data_.emplace_back(ros::Time::now());
}

void RateChecker::update()
{
  std::lock_guard<std::mutex> lock(mtx_);
  std::vector<ros::Time> buffer;
  for (const auto& el : data_)
  {
    if (el + ros::Duration(buffer_duration_) > ros::Time::now())
    {
      buffer.emplace_back(el);
    }
  }
  data_ = buffer;
}

boost::optional<double> RateChecker::getRate()
{
  boost::optional<double> rate;
  if (ros::Time::now() < start_time_ + ros::Duration(buffer_duration_))
  {
    return boost::none;
  }
  update();
  std::lock_guard<std::mutex> lock(mtx_);
  rate = data_.size() / buffer_duration_;
  return rate;
}
}  // namespace autoware_health_checker
