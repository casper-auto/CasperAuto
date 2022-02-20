/*
 * Copyright 2019 Autoware Foundation
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *    http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TRAFFICLIGHT_RECOGNIZER_REGION_TLR_MXNET_REGION_TLR_MXNET_H
#define TRAFFICLIGHT_RECOGNIZER_REGION_TLR_MXNET_REGION_TLR_MXNET_H

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <autoware_msgs/Signals.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

#include "trafficlight_recognizer/context.h"
#include "trafficlight_recognizer/region_tlr_mxnet/mxnet_traffic_light_recognizer.h"

class BufferFile
{
public:
  std::string file_path_;
  int length_;
  char* buffer_;

  explicit BufferFile(std::string file_path) : file_path_(file_path)
  {
    std::ifstream ifs(file_path.c_str(), std::ios::in | std::ios::binary);
    if (!ifs)
    {
      ROS_ERROR("Can't open the file. Please check %s", file_path.c_str());
      length_ = 0;
      buffer_ = NULL;
      return;
    }

    ifs.seekg(0, std::ios::end);
    length_ = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    ROS_INFO("%s .. %d bytes", file_path.c_str(), length_);

    buffer_ = new char[sizeof(char) * length_];
    ifs.read(buffer_, length_);
    ifs.close();
  }

  int GetLength()
  {
    return length_;
  }
  char* GetBuffer()
  {
    return buffer_;
  }

  ~BufferFile()
  {
    if (buffer_)
    {
      delete[] buffer_;
      buffer_ = NULL;
    }
  }
};

class RegionTLRMxNetROSNode
{
public:
  RegionTLRMxNetROSNode();

  ~RegionTLRMxNetROSNode();

  void RunRecognition();

  void ImageRawCallback(const sensor_msgs::Image& image);

  void ROISignalCallback(const autoware_msgs::Signals::ConstPtr& extracted_pos);

  // The vector of data structure to save traffic light state, position, ...etc
  std::vector<Context> contexts_;

private:
  /* Light state transition probably happen in Japanese traffic light */
  const LightState kStateTransitionMatrix[4][4] =
  {
    /* current: */
    /* GREEN   , YELLOW    , RED    , UNDEFINED  */
    /* -------------------------------------------  */
    { GREEN, YELLOW, YELLOW, GREEN },   /* | previous = GREEN */
    { UNDEFINED, YELLOW, RED, YELLOW }, /* | previous = YELLOW */
    { GREEN, RED, RED, RED },           /* | previous = RED */
    { GREEN, YELLOW, RED, UNDEFINED }   /* | previous = UNDEFINED */
  };

  void GetROSParam();

  void StartSubscribersAndPublishers();

  void DetermineState(LightState in_current_state, Context* in_out_signal_context);

  void PublishTrafficLight(std::vector<Context> contexts);

  void PublishString(std::vector<Context> contexts);

  void PublishMarkerArray(std::vector<Context> contexts);

  void PublishImage(std::vector<Context> contexts);

  void SuperimposeCb(const std_msgs::Bool::ConstPtr& config_msg);

  // Execution parameter
  std::string image_topic_name_;
  std::string network_definition_file_name_;
  std::string pretrained_model_file_name_;
  bool use_gpu_;
  int gpu_id_;
  double score_threshold_;

  int change_state_threshold_;  // The threshold of state detected times to accept the state change

  // Subscribers
  ros::Subscriber image_subscriber;
  ros::Subscriber roi_signal_subscriber;
  ros::Subscriber superimpose_sub;

  // Publishers
  ros::Publisher signal_state_publisher;
  ros::Publisher signal_state_string_publisher;
  ros::Publisher marker_publisher;
  ros::Publisher superimpose_image_publisher;

  // Flag to show topic will be published in latch manner
  bool kAdvertiseInLatch_;

  // A frame image acquired from topic
  cv::Mat frame_;

  // Timestamp of a frame in process
  std_msgs::Header frame_header_;

  // The instance of the core class of traffic light recognition
  MxNetTrafficLightRecognizer recognizer;

  // constant values to pass recognition states to other nodes
  const int32_t kTrafficLightRed;
  const int32_t kTrafficLightGreen;
  const int32_t kTrafficLightUnknown;
  const std::string kStringRed;
  const std::string kStringGreen;
  const std::string kStringUnknown;
};

#endif  // TRAFFICLIGHT_RECOGNIZER_REGION_TLR_MXNET_REGION_TLR_MXNET_H
