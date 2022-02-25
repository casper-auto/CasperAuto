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

#include <ros/ros.h>
#include <string>
#include <gtest/gtest.h>
#include <ros_observer/lib_ros_observer.h>

using boost::interprocess::managed_shared_memory;
using boost::interprocess::shared_memory_object;
using boost::interprocess::scoped_lock;
using boost::interprocess::create_only;
using boost::interprocess::interprocess_mutex;

class MyShmDRStopRequest : public ShmDRStopRequest
{
  friend class ShmTestSuite;

public:
  MyShmDRStopRequest(): ShmDRStopRequest() {}
};

class MyShmVitalMonitor : public ShmVitalMonitor
{
  friend class ShmTestSuite;

public:
  MyShmVitalMonitor(std::string name, const double rate, VitalMonitorMode mode = VitalMonitorMode::CNT_CLEAR):
  ShmVitalMonitor(name, rate, mode) {}
};

class ShmTestSuite : public ::testing::Test
{
protected:
  std::unique_ptr<MyShmVitalMonitor> myVMObj_;
  std::unique_ptr<MyShmDRStopRequest> myDRObj_;

  void setupVM(std::string name, const double rate, VitalMonitorMode mode = VitalMonitorMode::CNT_CLEAR)
  {
    myVMObj_ = std::unique_ptr<MyShmVitalMonitor>(new MyShmVitalMonitor(name, rate, mode));
  }

  void setupDR(void)
  {
    myDRObj_ = std::unique_ptr<MyShmDRStopRequest>(new MyShmDRStopRequest());
  }

public:
  void modeTestVM(void)
  {
    ASSERT_EQ(myVMObj_->mode_, VitalMonitorMode::CNT_CLEAR);
    myVMObj_->set_mode(VitalMonitorMode::CNT_MON);
    ASSERT_EQ(myVMObj_->mode_, VitalMonitorMode::CNT_MON);
  }

  void nameTestVM(void)
  {
    ASSERT_EQ(myVMObj_->shm_name_, "SHM_NameTest");
    ASSERT_EQ(myVMObj_->mut_name_, "MUT_NameTest");
  }

  void openTestVM(void)
  {
    EXPECT_FALSE(myVMObj_->attempt_to_open());

    managed_shared_memory shm_new(create_only, SHM_NAME, SHM_SIZE);
    ShmVitalCounter*  p_cnt_new = shm_new.construct<ShmVitalCounter>(myVMObj_->shm_name_.c_str())();
    interprocess_mutex* p_mut_new = shm_new.construct<interprocess_mutex>(myVMObj_->mut_name_.c_str())();

    ASSERT_TRUE(myVMObj_->attempt_to_open());

    shared_memory_object::remove(SHM_NAME);
  }

  void runTestVM(void)
  {
    managed_shared_memory shm_new(create_only, SHM_NAME, SHM_SIZE);
    ShmVitalCounter*  p_cnt_new = shm_new.construct<ShmVitalCounter>(myVMObj_->shm_name_.c_str())();
    interprocess_mutex* p_mut_new = shm_new.construct<interprocess_mutex>(myVMObj_->mut_name_.c_str())();

    myVMObj_->run();
    ASSERT_TRUE(p_cnt_new->activated);

    shared_memory_object::remove(SHM_NAME);
  }

  void updateTestVM(void)
  {
    managed_shared_memory shm_new(create_only, SHM_NAME, SHM_SIZE);
    ShmVitalCounter*  p_cnt_new = shm_new.construct<ShmVitalCounter>(myVMObj_->shm_name_.c_str())();
    interprocess_mutex* p_mut_new = shm_new.construct<interprocess_mutex>(myVMObj_->mut_name_.c_str())();

    myVMObj_->mode_ = VitalMonitorMode::CNT_CLEAR;
    p_cnt_new->activated = true;
    p_cnt_new->value = 100;
    myVMObj_->update_vital_counter();
    ASSERT_EQ(p_cnt_new->value, 0);

    myVMObj_->mode_ = VitalMonitorMode::CNT_MON;
    myVMObj_->update_vital_counter();
    ASSERT_EQ(p_cnt_new->value, myVMObj_->polling_interval_msec_);

    p_cnt_new->value = SHM_COUNTER_MAX;
    myVMObj_->update_vital_counter();
    ASSERT_EQ(p_cnt_new->value, SHM_COUNTER_MAX);

    p_cnt_new->value = p_cnt_new->thresh - 10;  /* 100hz */
    myVMObj_->update_vital_counter();
    ASSERT_EQ(p_cnt_new->modstatus, ModuleStatus::Normal);
    myVMObj_->update_vital_counter();
    ASSERT_EQ(p_cnt_new->modstatus, ModuleStatus::ErrorDetected);

    shared_memory_object::remove(SHM_NAME);
  }

  void errorDetectionTestVM(void)
  {
    ASSERT_FALSE(myVMObj_->is_opened_);

    managed_shared_memory shm_new(create_only, SHM_NAME, SHM_SIZE);
    ShmVitalCounter*  p_cnt_new = shm_new.construct<ShmVitalCounter>(myVMObj_->shm_name_.c_str())();
    interprocess_mutex* p_mut_new = shm_new.construct<interprocess_mutex>(myVMObj_->mut_name_.c_str())();

    myVMObj_->run();
    p_cnt_new->modstatus = ModuleStatus::Normal;
    ASSERT_FALSE(myVMObj_->is_error_detected());
    p_cnt_new->modstatus = ModuleStatus::ErrorDetected;
    ASSERT_TRUE(myVMObj_->is_error_detected());

    shared_memory_object::remove(SHM_NAME);
  }

  void openTestDR(void)
  {
    myDRObj_->is_opened_ = myDRObj_->attempt_to_open();
    ASSERT_FALSE(myDRObj_->is_opened_);

    managed_shared_memory shm_new(create_only, SHM_NAME, SHM_SIZE);
    bool*  p_stop_request_new = shm_new.construct<bool>(myDRObj_->shm_name_.c_str())();
    interprocess_mutex* p_mut_new = shm_new.construct<interprocess_mutex>(myDRObj_->mut_name_.c_str())();

    myDRObj_->is_opened_ = myDRObj_->attempt_to_open();
    ASSERT_TRUE(myDRObj_->is_opened_);

    shared_memory_object::remove(SHM_NAME);
  }

  void clearTestDR(void)
  {
    managed_shared_memory shm_new(create_only, SHM_NAME, SHM_SIZE);
    bool*  p_stop_request_new = shm_new.construct<bool>(myDRObj_->shm_name_.c_str())();
    interprocess_mutex* p_mut_new = shm_new.construct<interprocess_mutex>(myDRObj_->mut_name_.c_str())();

    (*p_stop_request_new) = false;
    myDRObj_->clear_request();
    ASSERT_FALSE((*p_stop_request_new));
    (*p_stop_request_new) = true;
    myDRObj_->clear_request();
    ASSERT_FALSE((*p_stop_request_new));

    shared_memory_object::remove(SHM_NAME);
  }

  void requestCheckTestDR(void)
  {
    managed_shared_memory shm_new(create_only, SHM_NAME, SHM_SIZE);
    bool*  p_stop_request_new = shm_new.construct<bool>(myDRObj_->shm_name_.c_str())();
    interprocess_mutex* p_mut_new = shm_new.construct<interprocess_mutex>(myDRObj_->mut_name_.c_str())();
    bool is_request_received = false;

    (*p_stop_request_new) = false;
    is_request_received = myDRObj_->is_request_received();
    ASSERT_FALSE(is_request_received);
    (*p_stop_request_new) = true;
    is_request_received = myDRObj_->is_request_received();
    ASSERT_TRUE(is_request_received);

    shared_memory_object::remove(SHM_NAME);
  }
};

TEST_F(ShmTestSuite, ModeTestVM)
{
  setupVM("ModeTest", 100.0);
  modeTestVM();
}

TEST_F(ShmTestSuite, NameTestVM)
{
  setupVM("NameTest", 100.0);
  nameTestVM();
}

TEST_F(ShmTestSuite, OpenTestVM)
{
  setupVM("OpenTest", 100.0);
  openTestVM();
}

TEST_F(ShmTestSuite, RunTestVM)
{
  setupVM("RunTest", 100.0);
  runTestVM();
}

TEST_F(ShmTestSuite, UpdateTestVM)
{
  setupVM("UpdateTest", 100.0);
  updateTestVM();
}

TEST_F(ShmTestSuite, ErrorDetectionTestVM)
{
  setupVM("ErrorDetectionTest", 100.0);
  errorDetectionTestVM();
}

TEST_F(ShmTestSuite, OpenTestDR)
{
  setupDR();
  openTestDR();
}

TEST_F(ShmTestSuite, ClearTestDR)
{
  setupDR();
  clearTestDR();
}

TEST_F(ShmTestSuite, RequestCheckTestDR)
{
  setupDR();
  requestCheckTestDR();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "SharedMemoryMonitorTestNode");

  return RUN_ALL_TESTS();
}
