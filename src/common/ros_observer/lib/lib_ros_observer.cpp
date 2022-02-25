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
#include <algorithm>
#include <iostream>
#include <ros_observer/lib_ros_observer.h>
#include <ros_observer/ros_observer.h>

using boost::interprocess::managed_shared_memory;
using boost::interprocess::scoped_lock;
using boost::interprocess::open_only;
using boost::interprocess::interprocess_mutex;
using boost::interprocess::interprocess_exception;

ShmVitalMonitor::ShmVitalMonitor(std::string mod_name, const double loop_rate, VitalMonitorMode mode) :
  is_opened_(false), name_(mod_name), shm_name_("SHM_" + mod_name), mut_name_("MUT_" + mod_name),
  mode_(mode), polling_interval_msec_(1000.0/loop_rate) {}

void ShmVitalMonitor::run(void)
{
  if (is_opened_)
  {
    update_vital_counter();
  }
  else
  {
    is_opened_ = attempt_to_open();
    if (is_opened_) init_vital_counter();
  }
}

void ShmVitalMonitor::init_vital_counter(void)
{
  if (mode_ == VitalMonitorMode::CNT_CLEAR)
  {
    try
    {
      managed_shared_memory shm(open_only, SHM_NAME);
      ShmVitalCounter* p_cnt = shm.find<ShmVitalCounter>(shm_name_.c_str()).first;
      interprocess_mutex* p_mut = shm.find<interprocess_mutex>(mut_name_.c_str()).first;
      scoped_lock<interprocess_mutex> scpdlock(*p_mut);

      p_cnt->activated = true;
      p_cnt->thresh = (polling_interval_msec_)*(SHM_TH_COUNTER);
      p_cnt->value = 0;
    }
    catch(interprocess_exception &ex)
    {
      std::cout << "[INFO][Failed to connect shared memory]" << std::endl;
    }
  }
}

void ShmVitalMonitor::update_vital_counter(void)
{
  try
  {
    managed_shared_memory shm(open_only, SHM_NAME);
    ShmVitalCounter* p_cnt = shm.find<ShmVitalCounter>(shm_name_.c_str()).first;
    interprocess_mutex* p_mut = shm.find<interprocess_mutex>(mut_name_.c_str()).first;
    scoped_lock<interprocess_mutex> scpdlock(*p_mut);

    if (mode_ == VitalMonitorMode::CNT_CLEAR)
    {
      p_cnt->value = 0;
    }
    else if (mode_ == VitalMonitorMode::CNT_MON)
    {
      p_cnt->value = (p_cnt->activated) ? std::min((p_cnt->value + polling_interval_msec_), SHM_COUNTER_MAX) : 0;
      p_cnt->modstatus = (p_cnt->value > p_cnt->thresh) ? ModuleStatus::ErrorDetected : ModuleStatus::Normal;
    }
  }
  catch(interprocess_exception &ex)
  {
    std::cout << "[INFO][Failed to connect shared memory]" << std::endl;
  }
}

bool ShmVitalMonitor::attempt_to_open(void)
{
  bool is_opened = false;

  try
  {
    managed_shared_memory shm(open_only, SHM_NAME);
    ShmVitalCounter* p_cnt = shm.find<ShmVitalCounter>(shm_name_.c_str()).first;
    interprocess_mutex* p_mut = shm.find<interprocess_mutex>(mut_name_.c_str()).first;
    is_opened = true;
  }
  catch(interprocess_exception &ex)
  {
    is_opened = false;
  }
  return is_opened;
}

bool ShmVitalMonitor::is_error_detected(void)
{
  bool is_error_detected = false;

  if (!is_opened_)
  {
    is_opened_ = attempt_to_open();
  }
  else
  {
    try
    {
      managed_shared_memory shm(open_only, SHM_NAME);
      ShmVitalCounter* p_cnt = shm.find<ShmVitalCounter>(shm_name_.c_str()).first;
      interprocess_mutex* p_mut = shm.find<interprocess_mutex>(mut_name_.c_str()).first;

      scoped_lock<interprocess_mutex> scpdlock(*p_mut);
      is_error_detected = (p_cnt->modstatus == ModuleStatus::ErrorDetected) ? true : false;
    }
    catch(interprocess_exception &ex)
    {
      is_error_detected = true;
    }
  }
  return is_error_detected;
}

bool ShmDRStopRequest::is_request_received(void)
{
  bool is_request_received = false;

  if (!is_opened_)
  {
    is_opened_ = attempt_to_open();
  }
  else
  {
    try
    {
      managed_shared_memory shm(open_only, SHM_NAME);
      bool* p_stop_request = shm.find<bool>(shm_name_.c_str()).first;
      interprocess_mutex* p_mut = shm.find<interprocess_mutex>(mut_name_.c_str()).first;

      scoped_lock<interprocess_mutex> scpdlock(*p_mut);
      is_request_received = (*p_stop_request);
    }
    catch(interprocess_exception &ex)
    {
      std::cout << "[INFO][Failed to connect shared memory]" << std::endl;
      is_request_received = false;
    }
  }
  return is_request_received;
}

void ShmDRStopRequest::clear_request(void)
{
  if (!is_opened_)
  {
    is_opened_ = attempt_to_open();
  }
  else
  {
    try
    {
      managed_shared_memory shm(open_only, SHM_NAME);
      bool* p_stop_request = shm.find<bool>(shm_name_.c_str()).first;
      interprocess_mutex* p_mut = shm.find<interprocess_mutex>(mut_name_.c_str()).first;

      scoped_lock<interprocess_mutex> scpdlock(*p_mut);
      (*p_stop_request) = false;
    }
    catch(interprocess_exception &ex)
    {
      std::cout << "[INFO][Failed to connect shared memory]" << std::endl;
    }
  }
}

bool ShmDRStopRequest::attempt_to_open(void)
{
  bool is_opened = false;

  try
  {
    managed_shared_memory shm(open_only, SHM_NAME);
    bool* p_stop_request = shm.find<bool>(shm_name_.c_str()).first;
    interprocess_mutex* p_mut = shm.find<interprocess_mutex>(mut_name_.c_str()).first;
    is_opened = true;
  }
  catch(interprocess_exception &ex)
  {
    is_opened = false;
  }
  return is_opened;
}
