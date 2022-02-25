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
#include <autoware_health_checker/health_analyzer/health_analyzer.h>

HealthAnalyzer::HealthAnalyzer(ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh), pnh_(pnh)
{
  using SystemStatus = autoware_system_msgs::SystemStatus;
  pnh_.param("warn_nodes_count_threshold", warn_nodes_count_threshold_, 30);
  system_status_summary_pub_ =
    nh_.advertise<SystemStatus>("system_status/summary", 1);
  system_status_sub_ = nh_.subscribe(
    "system_status", 1, &HealthAnalyzer::systemStatusCallback, this);
}

HealthAnalyzer::~HealthAnalyzer()
{
  writeDot();
}

int HealthAnalyzer::countWarn(const autoware_system_msgs::SystemStatus& msg)
{
  int count = 0;
  for (const auto& status_array_list : msg.node_status)
  {
    for (const auto& status_array : status_array_list.status)
    {
      for (const auto& status : status_array.status)
      {
        if (status.level == AwDiagStatus::WARN)
        {
          count++;
        }
      }
    }
  }
  for (const auto& status_array_list : msg.hardware_status)
  {
    for (const auto& status_array : status_array_list.status)
    {
      for (const auto& status : status_array.status)
      {
        if (status.level == AwDiagStatus::WARN)
        {
          count++;
        }
      }
    }
  }
  return count;
}

std::vector<std::string> HealthAnalyzer::findWarningNodes(
  const autoware_system_msgs::SystemStatus& sys_status)
{
  std::vector<std::string> ret;
  auto isOverWarn = [](autoware_health_checker::ErrorLevel level)
  {
    return (
      level == AwDiagStatus::WARN ||
      level == AwDiagStatus::ERROR ||
      level == AwDiagStatus::FATAL);
  };
  for (const auto& node_status : sys_status.node_status)
  {
    for (const auto& status_array : node_status.status)
    {
      for (const auto& status : status_array.status)
      {
        if (isAlreadyExist(ret, node_status.node_name))
        {
          continue;
        }
        else if (isOverWarn(status.level))
        {
          ret.emplace_back(node_status.node_name);
        }
      }
    }
  }
  return ret;
}

std::vector<std::string> HealthAnalyzer::findErrorNodes(
  const autoware_system_msgs::SystemStatus& sys_status)
{
  std::vector<std::string> ret;
  auto isOverError = [](autoware_health_checker::ErrorLevel level)
  {
    return (
      level == AwDiagStatus::ERROR ||
      level == AwDiagStatus::FATAL);
  };
  for (const auto& node_status : sys_status.node_status)
  {
    for (const auto& status_array : node_status.status)
    {
      const auto& node_name = node_status.node_name;
      for (const auto& status : status_array.status)
      {
        if (isAlreadyExist(ret, node_name) || !isOverError(status.level))
        {
          continue;
        }
        if (isAlreadyExist(sys_status.available_nodes, node_name))
        {
          ret.emplace_back(node_name);
        }
      }
    }
  }
  return ret;
}

std::vector<std::string> HealthAnalyzer::findRootNodes(
  const std::vector<std::string>& target_nodes)
{
  std::vector<std::string> ret;
  for (const auto& node : target_nodes)
  {
    std::vector<std::string> pub_nodes;
    bool depend_found = false;
    adjacency_iterator_t vi, vi_end;
    auto vertex = getTargetNode(node);
    if (!vertex)
    {
      continue;
    }
    boost::tie(vi, vi_end) = adjacent_vertices(vertex.get(), depend_graph_);
    for (; vi != vi_end; ++vi)
    {
      if (isAlreadyExist(target_nodes, depend_graph_[*vi].node_name))
      {
        depend_found = true;
      }
    }
    if (!depend_found)
    {
      ret.emplace_back(node);
    }
  }
  return ret;
}

autoware_system_msgs::SystemStatus HealthAnalyzer::filterSystemStatus(
  const autoware_system_msgs::SystemStatus& status)
{
  int warn_count = countWarn(status);
  autoware_system_msgs::SystemStatus filtered_status(status);
  filtered_status.detect_too_match_warning =
    (warn_count >= warn_nodes_count_threshold_);
  filtered_status.node_status.clear();
  std::vector<std::string> root_nodes(findRootNodes(findWarningNodes(status)));
  for (const auto& node_status : status.node_status)
  {
    if (isAlreadyExist(root_nodes, node_status.node_name))
    {
      filtered_status.node_status.emplace_back(node_status);
    }
  }
  return filtered_status;
}

void HealthAnalyzer::systemStatusCallback(
    const autoware_system_msgs::SystemStatus::ConstPtr& msg)
{
  generateDependGraph(*msg);
  system_status_summary_pub_.publish(filterSystemStatus(*msg));
}

void HealthAnalyzer::generateDependGraph(
  const autoware_system_msgs::SystemStatus& status)
{
  depend_graph_ = graph_t();
  for (const auto& el : status.topic_statistics)
  {
    addDepend(el);
  }
}

void HealthAnalyzer::writeDot()
{
  const std::string path =
    ros::package::getPath("autoware_health_checker") + "/data/node_depends.dot";
  std::ofstream f(path.c_str());
  boost::write_graphviz(f, depend_graph_,
    boost::make_label_writer(get(&node_property::node_name, depend_graph_)));
}

boost::optional<HealthAnalyzer::vertex_t>
  HealthAnalyzer::getTargetNode(const std::string& target_node)
{
  vertex_t ret;
  auto vertex_range = boost::vertices(depend_graph_);
  for (auto first = vertex_range.first, last = vertex_range.second;
       first != last; ++first)
  {
    vertex_t v = *first;
    if (depend_graph_[v].node_name == target_node)
    {
      ret = v;
      return ret;
    }
  }
  return boost::none;
}

void HealthAnalyzer::addDepend(
  const rosgraph_msgs::TopicStatistics& statistics)
{
  if (statistics.node_pub == statistics.node_sub)
  {
    return;
  }
  vertex_t node_sub;
  vertex_t node_pub;
  edge_t topic;
  auto pub_vertex = getTargetNode(statistics.node_pub);
  auto sub_vertex = getTargetNode(statistics.node_sub);
  if (!pub_vertex)
  {
    pub_vertex = boost::add_vertex(depend_graph_);
    depend_graph_[pub_vertex.get()].node_name = statistics.node_pub;
  }
  if (!sub_vertex)
  {
    sub_vertex = boost::add_vertex(depend_graph_);
    depend_graph_[sub_vertex.get()].node_name = statistics.node_sub;
  }
  bool inserted = false;
  boost::tie(topic, inserted) =
    boost::add_edge(sub_vertex.get(), pub_vertex.get(), depend_graph_);
  depend_graph_[topic].node_sub = statistics.node_sub;
  depend_graph_[topic].node_pub = statistics.node_pub;
}
