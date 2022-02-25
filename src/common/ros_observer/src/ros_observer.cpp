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

#include <csignal>
#include <chrono>
#include <iostream>
#include <string>
#include <algorithm>
#include <ros_observer/ros_observer.h>

using boost::interprocess::managed_shared_memory;
using boost::interprocess::interprocess_mutex;
using boost::interprocess::shared_memory_object;
using boost::interprocess::scoped_lock;
using boost::interprocess::create_only;

static void sig_handler_init(void);
static void sig_handler(void);

static constexpr double ROS_OBSERVE_MONITOR_RATE = 100;
static constexpr unsigned int POLLING_INTERVAL_MSEC = (1000.0 / ROS_OBSERVE_MONITOR_RATE);
static constexpr unsigned int POLLING_INTERVAL_USEC = (POLLING_INTERVAL_MSEC * 1000);
static constexpr unsigned int SHM_TH_COUNTER_RO = 10;

static bool terminate_req_rcvd = false;

// Signal Handler
static void sig_handler(int signo)
{
  terminate_req_rcvd = true;
}

// Signal Handler Initialization
static void sig_handler_init(void)
{
  if (signal(SIGUSR1, sig_handler) == SIG_ERR)
  {
    std::cout << "[INFO][Cannot catch SIGUSR1]" << std::endl;
  }

  if (signal(SIGTERM, sig_handler) == SIG_ERR)
  {
    std::cout << "[INFO][Cannot catch SIGTERM]" << std::endl;
  }

  if (signal(SIGINT, sig_handler) == SIG_ERR)
  {
    std::cout << "[INFO][Cannot catch SIGINT]" << std::endl;
  }

  if (signal(SIGQUIT, sig_handler) == SIG_ERR)
  {
    std::cout << "[INFO][Cannot catch SIGQUIT]" << std::endl;
  }
}

int main(int argc, char* argv[])
{
  sig_handler_init();

  struct tm localtime;
  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);
  localtime_r(&now_c, &localtime);
  std::cout << "[START][TIME][LOCAL: " << std::put_time(&localtime, "%c") << "]" << std::endl;

  shared_memory_object::remove(SHM_NAME);
  managed_shared_memory shm(create_only, SHM_NAME, SHM_SIZE);

  ShmVitalCounter* p_cnt_RO = shm.construct<ShmVitalCounter>("SHM_RosObserver")();
  ShmVitalCounter* p_cnt_HA = shm.construct<ShmVitalCounter>("SHM_HealthAggregator")();
  ShmVitalCounter* p_cnt_EH = shm.construct<ShmVitalCounter>("SHM_EmergencyHandler")();
  ShmVitalCounter* p_cnt_TG = shm.construct<ShmVitalCounter>("SHM_TwistGate")();
  ShmVitalCounter* p_cnt_YMC = shm.construct<ShmVitalCounter>("SHM_YMC_VehicleDriver")();
  ShmVitalCounter* p_cnt_AS = shm.construct<ShmVitalCounter>("SHM_AS_VehicleDriver")();
  bool* p_stopReq_DR = shm.construct<bool>("SHM_DRStopRequest")();

  interprocess_mutex* p_mut_RO = shm.construct<interprocess_mutex>("MUT_RosObserver")();
  interprocess_mutex* p_mut_HA = shm.construct<interprocess_mutex>("MUT_HealthAggregator")();
  interprocess_mutex* p_mut_EH = shm.construct<interprocess_mutex>("MUT_EmergencyHandler")();
  interprocess_mutex* p_mut_TG = shm.construct<interprocess_mutex>("MUT_TwistGate")();
  interprocess_mutex* p_mut_YMC = shm.construct<interprocess_mutex>("MUT_YMC_VehicleDriver")();
  interprocess_mutex* p_mut_AS = shm.construct<interprocess_mutex>("MUT_AS_VehicleDriver")();
  interprocess_mutex* p_mut_DR = shm.construct<interprocess_mutex>("MUT_DRStopRequest")();

  {
    scoped_lock<interprocess_mutex> scpdlock_RO(*p_mut_RO);
    p_cnt_RO->activated = true;
    p_cnt_RO->thresh = (POLLING_INTERVAL_MSEC) * (SHM_TH_COUNTER_RO);
    p_cnt_RO->value = 0;
  }

  while (!terminate_req_rcvd)
  {
    {
      scoped_lock<interprocess_mutex> scpdlock_RO(*p_mut_RO);
      scoped_lock<interprocess_mutex> scpdlock_HA(*p_mut_HA);
      scoped_lock<interprocess_mutex> scpdlock_EH(*p_mut_EH);
      scoped_lock<interprocess_mutex> scpdlock_TG(*p_mut_TG);
      scoped_lock<interprocess_mutex> scpdlock_YMC(*p_mut_YMC);
      scoped_lock<interprocess_mutex> scpdlock_AS(*p_mut_AS);
      scoped_lock<interprocess_mutex> scpdlock_DR(*p_mut_DR);

      p_cnt_RO->value = 0;
      p_cnt_HA->value =
          (p_cnt_HA->activated) ? std::min((p_cnt_HA->value + (POLLING_INTERVAL_MSEC)), SHM_COUNTER_MAX) : 0;
      p_cnt_EH->value =
          (p_cnt_EH->activated) ? std::min((p_cnt_EH->value + (POLLING_INTERVAL_MSEC)), SHM_COUNTER_MAX) : 0;
      p_cnt_TG->value =
          (p_cnt_TG->activated) ? std::min((p_cnt_TG->value + (POLLING_INTERVAL_MSEC)), SHM_COUNTER_MAX) : 0;
      p_cnt_YMC->value =
          (p_cnt_YMC->activated) ? std::min((p_cnt_YMC->value + (POLLING_INTERVAL_MSEC)), SHM_COUNTER_MAX) : 0;
      p_cnt_AS->value =
          (p_cnt_AS->activated) ? std::min((p_cnt_AS->value + (POLLING_INTERVAL_MSEC)), SHM_COUNTER_MAX) : 0;

      static bool ros_error_detected_prev = false;
      bool ros_error_detected = false;
      std::string error_node;

      if (p_cnt_HA->value > p_cnt_HA->thresh)
      {
        ros_error_detected = true;
        error_node = "Health Aggregator";
      }

      if (p_cnt_EH->value > p_cnt_EH->thresh)
      {
        ros_error_detected = true;
        error_node = "Emergency Handler";
      }

      if (p_cnt_TG->value > p_cnt_TG->thresh)
      {
        ros_error_detected = true;
        error_node = "Twist Gate";
      }

      if (p_cnt_YMC->value > p_cnt_YMC->thresh)
      {
        ros_error_detected = true;
        error_node = "YMC Vehicle Driver";
      }

      if (p_cnt_AS->value > p_cnt_AS->thresh)
      {
        ros_error_detected = true;
        error_node = "AS Vehicle Driver";
      }

      if (ros_error_detected)
      {
        p_cnt_HA->modstatus = ModuleStatus::ErrorDetected;
        if (!ros_error_detected_prev)
        {
          (*p_stopReq_DR) = true;
        }

        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        localtime_r(&now_c, &localtime);
        std::cerr << "[START][TIME][LOCAL: " << std::put_time(&localtime, "%c") << "][" << error_node.c_str() << "]"
                  << std::endl;
      }
      else
      {
        p_cnt_HA->modstatus = ModuleStatus::Normal;
        if (ros_error_detected_prev)
        {
          (*p_stopReq_DR) = false;
        }
      }
      ros_error_detected_prev = ros_error_detected;
    }
    usleep(POLLING_INTERVAL_USEC);
  }

  shared_memory_object::remove(SHM_NAME);

  return 0;
}
