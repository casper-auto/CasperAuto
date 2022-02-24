#include <iostream>
#include <vector>
#include <map>

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <casper_auto_msgs/MapObject.h>
#include <casper_auto_msgs/MapBasepath.h>
#include <casper_auto_msgs/MapLane.h>
#include <casper_auto_msgs/MapBoundary.h>
#include <casper_auto_msgs/MapTrafficLight.h>
#include <casper_auto_msgs/MapCrosswalk.h>
#include <casper_auto_msgs/MapStopLine.h>

#include <casper_auto_msgs/VehicleState.h>

#include <odr_map.h>

class OdrMapLoader {
public:

  OdrMapLoader(ros::NodeHandle nh, std::shared_ptr<OdrMap> map_parser);

private:

  void initalize();

  std::vector<geometry_msgs::Point> convertGeometryPoints(std::vector<geometry_msgs::Point32> &points);

  visualization_msgs::Marker createPathIndexMarkerMsg(std::vector<geometry_msgs::Point> &points, int marker_id);
  visualization_msgs::Marker convertToBasePathMarkerMsg(std::vector<geometry_msgs::Point> &points, int marker_id);
  visualization_msgs::Marker convertToLaneBoundaryMarkerMsg(std::vector<geometry_msgs::Point> &points, int marker_id);
  visualization_msgs::Marker convertToRoadBoundaryMarkerMsg(std::vector<geometry_msgs::Point> &points, int marker_id);

  void timerCallback(const ros::TimerEvent &e);

private:

  ros::NodeHandle nh_;

  // ros timer
  ros::Timer timer_;

  // ROS Publishers
  ros::Publisher path_index_marker_pub_;
  ros::Publisher base_path_marker_pub_;
  ros::Publisher lane_boundary_marker_pub_;
  ros::Publisher road_boundary_marker_pub_;

  // MarkerArray
  visualization_msgs::MarkerArrayPtr path_index_msg_;
  visualization_msgs::MarkerArrayPtr basepath_msg_;
  visualization_msgs::MarkerArrayPtr lane_boundaries_msg_;
  visualization_msgs::MarkerArrayPtr road_boundaries_msg_;

  // map_parser
  std::shared_ptr<OdrMap> map_parser_;

  // map objects
  std::vector<casper_auto_msgs::MapBasepath> map_basepaths_;
  std::vector<casper_auto_msgs::MapLane> map_lane_boundaries_;
  std::vector<casper_auto_msgs::MapBoundary> map_road_boundaries_;
};

OdrMapLoader::OdrMapLoader(ros::NodeHandle nh, std::shared_ptr<OdrMap> map_parser) : nh_(nh), map_parser_(map_parser) {
  initalize();
}

void OdrMapLoader::initalize() {
  //////////////////////////////////////////////////////////////////////////////
  // Initalize Publishers
  //////////////////////////////////////////////////////////////////////////////
  path_index_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/casper_auto/odr_map/path_index_marker", 1);
  base_path_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/casper_auto/odr_map/base_path_marker", 1);
  lane_boundary_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/casper_auto/odr_map/lane_boundary_marker", 1);
  road_boundary_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/casper_auto/odr_map/road_boundary_marker", 1);

  // marker ptr
  path_index_msg_.reset(new visualization_msgs::MarkerArray);
  basepath_msg_.reset(new visualization_msgs::MarkerArray);
  lane_boundaries_msg_.reset(new visualization_msgs::MarkerArray);
  road_boundaries_msg_.reset(new visualization_msgs::MarkerArray);

  //////////////////////////////////////////////////////////////////////////
  // Get All Map Info
  //////////////////////////////////////////////////////////////////////////
  casper_auto_msgs::MapObject map_obj = map_parser_->GetAllMapObjects();
  map_basepaths_ = map_obj.basepaths;
  map_lane_boundaries_ = map_obj.lanes;
  map_road_boundaries_ = map_obj.boundaries;

  //////////////////////////////////////////////////////////////////////////
  // MarkerArray msg
  //////////////////////////////////////////////////////////////////////////
  path_index_msg_->markers.resize(map_basepaths_.size());
  basepath_msg_->markers.resize(map_basepaths_.size());
  for(int i = 0; i < map_basepaths_.size(); i++) {
    casper_auto_msgs::MapBasepath basepath = map_basepaths_[i];
    int path_index = basepath.id.unique_id;
    std::vector<geometry_msgs::Point> points = convertGeometryPoints(basepath.points);
    path_index_msg_->markers[i] = createPathIndexMarkerMsg(points, path_index);
    basepath_msg_->markers[i] = convertToBasePathMarkerMsg(points, i+1);
  }

  lane_boundaries_msg_->markers.resize(map_lane_boundaries_.size());
  for(int i = 0; i < map_lane_boundaries_.size(); i++) {
    casper_auto_msgs::MapLane map_lane = map_lane_boundaries_[i];
    std::vector<casper_auto_msgs::LaneInformation> lane_info_array = map_lane.lane_information;
    std::vector<geometry_msgs::Point> points;
    for(auto &lane_info : lane_info_array) {
      std::vector<geometry_msgs::Point> temp = convertGeometryPoints(lane_info.points);
      points.reserve(points.size() + temp.size());
      points.insert(points.end(), temp.begin(), temp.end());
    }
    lane_boundaries_msg_->markers[i] = convertToLaneBoundaryMarkerMsg(points, map_lane.id.unique_id);
  }

  road_boundaries_msg_->markers.resize(map_road_boundaries_.size());
  for(int i = 0; i < map_road_boundaries_.size(); i++) {
    casper_auto_msgs::MapBoundary boundary = map_road_boundaries_[i];
    std::vector<geometry_msgs::Point> points = convertGeometryPoints(boundary.points);
    road_boundaries_msg_->markers[i] = convertToRoadBoundaryMarkerMsg(points, boundary.id.unique_id);
  }

  // createTimer
  timer_ = nh_.createTimer(ros::Duration(0.1), &OdrMapLoader::timerCallback, this);
}

std::vector<geometry_msgs::Point> OdrMapLoader::convertGeometryPoints(std::vector<geometry_msgs::Point32> &points) {
  std::vector<geometry_msgs::Point> new_points;
  for(auto &p : points) {
    geometry_msgs::Point new_p;
    new_p.x = p.x;
    new_p.y = p.y;
    new_p.z = p.z;
    new_points.push_back(new_p);
  }
  return new_points;
}

visualization_msgs::Marker OdrMapLoader::createPathIndexMarkerMsg(std::vector<geometry_msgs::Point> &points, int marker_id) {
  visualization_msgs::Marker path_index_marker;
  path_index_marker.header.frame_id = "map";
  // path_index_marker.header.stamp = ros::Time::now();
  path_index_marker.ns = "path_index";
  path_index_marker.id = marker_id;
  path_index_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  path_index_marker.action = visualization_msgs::Marker::ADD;
  path_index_marker.pose.orientation.w = 1.0;

  // text
  path_index_marker.text = std::to_string(marker_id);

  // position
  path_index_marker.pose.position = points[points.size()/2];

  // Size
  path_index_marker.scale.z = 1.0;

  // Color
  path_index_marker.color.r = 1.0;
  path_index_marker.color.g = 1.0;
  path_index_marker.color.b = 1.0;
  path_index_marker.color.a = 1.0;

  return path_index_marker;
}

visualization_msgs::Marker OdrMapLoader::convertToBasePathMarkerMsg(std::vector<geometry_msgs::Point> &points, int marker_id) {
  visualization_msgs::Marker base_path_marker;
  base_path_marker.header.frame_id = "map";
  // base_path_marker.header.stamp = ros::Time::now();
  base_path_marker.ns = "base_path";
  base_path_marker.id = marker_id;
  base_path_marker.type = visualization_msgs::Marker::LINE_STRIP; // or POINTS, LINE_STRIP
  base_path_marker.action = visualization_msgs::Marker::ADD;
  base_path_marker.pose.orientation.w = 1.0;

  // Size
  base_path_marker.scale.x = 1.0;

  // Color
  base_path_marker.color.r = 1.0;
  base_path_marker.color.g = 1.0;
  base_path_marker.color.b = 1.0;
  base_path_marker.color.a = 0.2;

  base_path_marker.points = points;

  return base_path_marker;
}

visualization_msgs::Marker OdrMapLoader::convertToLaneBoundaryMarkerMsg(std::vector<geometry_msgs::Point> &points, int marker_id) {
  visualization_msgs::Marker boundary_marker;
  boundary_marker.header.frame_id = "map";
  // boundary_marker.header.stamp = ros::Time::now();
  boundary_marker.ns = "lane_boundaries";
  boundary_marker.id = marker_id;
  boundary_marker.type = visualization_msgs::Marker::POINTS; // or POINTS, LINE_STRIP
  boundary_marker.action = visualization_msgs::Marker::ADD;
  boundary_marker.pose.orientation.w = 1.0;

  // Size
  boundary_marker.scale.x = 0.25;
  boundary_marker.scale.y = 0.25;
  boundary_marker.scale.z = 0.25;

  // Color
  boundary_marker.color.r = 1.0;
  boundary_marker.color.g = 1.0;
  boundary_marker.color.b = 1.0;
  boundary_marker.color.a = 1.0;

  boundary_marker.points = points;

  return boundary_marker;
}

visualization_msgs::Marker OdrMapLoader::convertToRoadBoundaryMarkerMsg(std::vector<geometry_msgs::Point> &points, int marker_id) {
  visualization_msgs::Marker boundary_marker;
  boundary_marker.header.frame_id = "map";
  // boundary_marker.header.stamp = ros::Time::now();
  boundary_marker.ns = "road_boundaries";
  boundary_marker.id = marker_id;
  boundary_marker.type = visualization_msgs::Marker::LINE_STRIP; // or POINTS, LINE_STRIP
  boundary_marker.action = visualization_msgs::Marker::ADD;
  boundary_marker.pose.orientation.w = 1.0;

  // Size
  boundary_marker.scale.x = 0.25;
  boundary_marker.scale.y = 0.25;
  boundary_marker.scale.z = 0.25;

  // Color
  boundary_marker.color.r = 1.0;
  boundary_marker.color.g = 1.0;
  boundary_marker.color.b = 1.0;
  boundary_marker.color.a = 1.0;

  boundary_marker.points = points;

  return boundary_marker;
}

void OdrMapLoader::timerCallback(const ros::TimerEvent &e) {
  //////////////////////////////////////////////////////////////////////////
  // Get All Map Info and Enable Visualization
  //////////////////////////////////////////////////////////////////////////
  path_index_marker_pub_.publish(path_index_msg_);
  base_path_marker_pub_.publish(basepath_msg_);
  lane_boundary_marker_pub_.publish(lane_boundaries_msg_);
  road_boundary_marker_pub_.publish(road_boundaries_msg_);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "odr_map_loader");
  ros::NodeHandle nh;

  ////////////////////////////////////////////////////////////////////////////
  // ROS Param Server
  ////////////////////////////////////////////////////////////////////////////
  double map_step;

  nh.param<double>("map_step", map_step, 1.0);

  ////////////////////////////////////////////////////////////////////////////
  // Wait for opendrive map msg
  ////////////////////////////////////////////////////////////////////////////
  ros::Duration(1.0).sleep();

  boost::shared_ptr<std_msgs::String const> shared_map_msg = ros::topic::waitForMessage<std_msgs::String>("/carla/map", nh);
  std::string xml_str = shared_map_msg->data;

  ////////////////////////////////////////////////////////////////////////////
  // Data type conversion and value checking
  ////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<OdrMap> map_parser = std::make_shared<OdrMap>(xml_str, map_step);

  ////////////////////////////////////////////////////////////////////////////
  // Map Loader instance
  ////////////////////////////////////////////////////////////////////////////
  OdrMapLoader map_loader(nh, map_parser);

  ////////////////////////////////////////////////////////////////////////////
  // ROS Spin
  ////////////////////////////////////////////////////////////////////////////
  ros::spin();

  return 0;
}
