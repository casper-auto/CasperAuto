#ifndef ROS_OBSERVER_ROS_OBSERVER_H
#define ROS_OBSERVER_ROS_OBSERVER_H

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
 */

#include <iostream>
#include <string>
#include <functional>
#include <utility>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/thread.hpp>

static constexpr const char* SHM_NAME = "SharedMemoryForVitalMonitor";
static constexpr int SHM_SIZE = 65536;
static constexpr unsigned int SHM_TH_COUNTER = 3;
static constexpr unsigned int SHM_COUNTER_MAX = 10000;

enum class ModuleStatus
{
  Normal,
  ErrorDetected
};

struct ShmVitalCounter
{
  ModuleStatus modstatus;

  bool activated;
  unsigned int thresh;
  unsigned int value;

  ShmVitalCounter() :
  modstatus(ModuleStatus::Normal), activated(false), thresh(0), value(0) {}
};

#endif  // ROS_OBSERVER_ROS_OBSERVER_H
