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

#ifndef AUTOWARE_HEALTH_CHECKER_CONSTANTS_H
#define AUTOWARE_HEALTH_CHECKER_CONSTANTS_H

#include <string>
#include <autoware_system_msgs/DiagnosticStatus.h>

namespace autoware_health_checker
{
/**
 * \brief ErrorLevel means the seriousness of abnormalities.
 * This value is managed by autoware_system_msgs::DiagnosticStatus.
 */
using ErrorLevel = uint8_t;

/**
 * \brief ErrorType means the type of abnormalities.
 * This value is managed by autoware_system_msgs::DiagnosticStatus.
 */
using ErrorType = uint8_t;

/**
 * \brief ErrorKey uniquely identifies each self-monitoring target
 * for parameter management and troubleshooting.
 * e.g. "topic_rate_***_slow", "value_***_high", etc.
 */
using ErrorKey = std::string;

/**
 * \brief ThresholdType represents the threshold type
 * of the self-monitoring target.
 * e.g. "min", "max", and "range".
 */
using ThreshType = std::string;
constexpr double BUFFER_DURATION = 0.5;
constexpr double NODE_STATUS_UPDATE_RATE = 10.0;
constexpr double SYSTEM_UPDATE_RATE = 30.0;
}  // namespace autoware_health_checker

#endif  // AUTOWARE_HEALTH_CHECKER_CONSTANTS_H
