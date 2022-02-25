#ifndef ROS_OBSERVER_LIB_ROS_OBSERVER_H
#define ROS_OBSERVER_LIB_ROS_OBSERVER_H

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

#include <string>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/thread.hpp>

#include <ros_observer/ros_observer.h>

enum class VitalMonitorMode
{
  CNT_CLEAR,
  CNT_MON
};

class ShmVitalMonitor
{
public:
  ShmVitalMonitor(std::string mod_name, const double loop_rate, VitalMonitorMode mode = VitalMonitorMode::CNT_CLEAR);

  void set_mode(VitalMonitorMode mode) {mode_ = mode;}
  void run(void);
  bool is_error_detected(void);

protected:
  std::string name_, shm_name_, mut_name_;
  VitalMonitorMode mode_;
  bool is_opened_;
  const unsigned int polling_interval_msec_;

  bool attempt_to_open(void);
  void init_vital_counter(void);
  void update_vital_counter(void);
};

class ShmDRStopRequest
{
public:
  ShmDRStopRequest() :
  is_opened_(false), name_("DRStopRequest"), shm_name_("SHM_" + name_), mut_name_("MUT_" + name_) {}

  void clear_request(void);
  bool is_request_received(void);

protected:
  bool is_opened_;
  std::string name_, shm_name_, mut_name_;

  bool attempt_to_open(void);
};

#endif  // ROS_OBSERVER_LIB_ROS_OBSERVER_H
