#include <XmlRpcCpp.h>
#include <math.h> /* atan2 */
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

std::vector<double> rotate(const std::vector<double> &in, const double theta) {
  std::vector<double> out(2);
  double s = sin(theta);
  double c = cos(theta);
  out[0] = in[0] * c - in[1] * s;
  out[1] = in[0] * s + in[1] * c;
  return out;
}

std::vector<double> globalToLocal(const std::vector<double> &center,
                                  const double theta,
                                  const std::vector<double> &p) {
  std::vector<double> delta(2);
  delta[0] = p[0] - center[0];
  delta[1] = p[1] - center[1];
  return rotate(delta, -theta);
}

std::vector<double> localToGlobal(const std::vector<double> &center,
                                  const double theta,
                                  const std::vector<double> &p) {
  std::vector<double> out = rotate(p, theta);
  out[0] += center[0];
  out[1] += center[1];
  return out;
}

class TrafficInfoLoader {
public:
  TrafficInfoLoader(ros::NodeHandle nh);

private:
  // ros
  ros::NodeHandle nh_;

  // ros timer
  ros::Timer timer_;

  // ROS Publishers
  ros::Publisher stoplines_marker_pub_;
  ros::Publisher crosswalks_marker_pub_;
  ros::Publisher interested_lanes_marker_pub_;

  // MarkerArray
  visualization_msgs::MarkerArrayPtr stoplines_msg_;
  visualization_msgs::MarkerArrayPtr crosswalks_msg_;
  visualization_msgs::MarkerArrayPtr interested_lanes_msg_;

  // load from ros param file
  std::vector<std::vector<std::vector<double>>> stoplines_;
  std::vector<std::vector<std::vector<double>>> crosswalks_;
  std::vector<std::vector<std::vector<double>>> interested_lane_;

  ///////////////////////////////////////////////////////////////
  // Functions
  ///////////////////////////////////////////////////////////////

  visualization_msgs::Marker
  constructstoplineMarker(std::vector<std::vector<double>> &stopline, int id);

  visualization_msgs::Marker
  constructCrosswalkMarker(std::vector<std::vector<double>> &crosswalk, int id);

  void timerCallback(const ros::TimerEvent &e);
};

TrafficInfoLoader::TrafficInfoLoader(ros::NodeHandle nh) : nh_(nh) {

  // ros params
  XmlRpc::XmlRpcValue stop_lines;
  nh_.getParam("/scenario/stop_lines", stop_lines);

  /*To ensure the reading will happen if the data is provided in right format*/
  if (stop_lines.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    stoplines_.resize(stop_lines.size());
    for (int i = 0; i < stop_lines.size(); i++) {
      XmlRpc::XmlRpcValue stop_line = stop_lines[i];
      XmlRpc::XmlRpcValue end1 = stop_line[0];
      XmlRpc::XmlRpcValue end2 = stop_line[1];
      // ROS_INFO("End point 1 is: %f and %f", double(end1[0]),
      // double(end1[1])); ROS_INFO("End point 2 is: %f and %f",
      // double(end2[0]), double(end2[1]));
      std::vector<double> p1 = {double(end1[0]), double(end1[1])};
      std::vector<double> p2 = {double(end2[0]), double(end2[1])};
      stoplines_[i] = {p1, p2};
    }
  }

  XmlRpc::XmlRpcValue crosswalks;
  nh_.getParam("/scenario/crosswalks", crosswalks);

  /*To ensure the reading will happen if the data is provided in right format*/
  if (crosswalks.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    crosswalks_.resize(crosswalks.size());
    for (int i = 0; i < crosswalks.size(); i++) {
      XmlRpc::XmlRpcValue crosswalk = crosswalks[i];
      XmlRpc::XmlRpcValue end1 = crosswalk[0];
      XmlRpc::XmlRpcValue end2 = crosswalk[1];
      // ROS_INFO("End point 1 is: %f and %f", double(end1[0]),
      // double(end1[1])); ROS_INFO("End point 2 is: %f and %f",
      // double(end2[0]), double(end2[1]));
      std::vector<double> p1 = {double(end1[0]), double(end1[1])};
      std::vector<double> p2 = {double(end2[0]), double(end2[1])};
      crosswalks_[i] = {p1, p2};
    }
  }

  // ros publishers
  stoplines_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/casper_auto/odr_map/stoplines_marker", 10);
  crosswalks_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/casper_auto/odr_map/crosswalks_marker", 10);

  // initalize marker ptr
  stoplines_msg_.reset(new visualization_msgs::MarkerArray);
  crosswalks_msg_.reset(new visualization_msgs::MarkerArray);
  interested_lanes_msg_.reset(new visualization_msgs::MarkerArray);

  // createTimer
  timer_ = nh_.createTimer(ros::Duration(0.1),
                           &TrafficInfoLoader::timerCallback, this);
}

visualization_msgs::Marker TrafficInfoLoader::constructstoplineMarker(
    std::vector<std::vector<double>> &stopline, int id) {
  std::vector<geometry_msgs::Point> stopline_points;
  for (auto p : stopline) {
    geometry_msgs::Point point;
    point.x = p[0];
    point.y = p[1];
    stopline_points.push_back(point);
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "stopline";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.points = stopline_points;
  marker.scale.x = 0.25;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  return marker;
}

// Construct crosswalk
visualization_msgs::Marker TrafficInfoLoader::constructCrosswalkMarker(
    std::vector<std::vector<double>> &crosswalk, int id) {
  // get crosswalk polygon
  std::vector<double> pl = crosswalk[0];
  std::vector<double> pr = crosswalk[1];

  // vector angle
  double alpha = atan2(pr[1] - pl[1], pr[0] - pl[0]);
  while (alpha > M_PI)
    alpha -= M_PI * 2;
  while (alpha < -M_PI)
    alpha += M_PI * 2;

  // transform to local frame, pl as center
  std::vector<double> pl_local{0, 0};
  std::vector<double> pr_local = globalToLocal(pl, alpha, pr);

  // find the polygon points
  std::vector<double> p1_local = {pl_local[0] - 2.0, pl_local[1] - 1.0};
  std::vector<double> p2_local = {pr_local[0] + 2.0, pr_local[1] - 1.0};
  std::vector<double> p3_local = {pl_local[0] - 2.0, pl_local[1] + 1.0};
  std::vector<double> p4_local = {pr_local[0] + 2.0, pr_local[1] + 1.0};

  // transform to global
  std::vector<double> p1 = localToGlobal(pl, alpha, p1_local);
  std::vector<double> p2 = localToGlobal(pl, alpha, p2_local);
  std::vector<double> p3 = localToGlobal(pl, alpha, p3_local);
  std::vector<double> p4 = localToGlobal(pl, alpha, p4_local);

  std::vector<geometry_msgs::Point> crosswalk_points;
  for (auto p : {p1, p2, p3, p4}) {
    geometry_msgs::Point point;
    point.x = p[0];
    point.y = p[1];
    crosswalk_points.push_back(point);
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "crosswalk";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.points = crosswalk_points;
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  return marker;
}

void TrafficInfoLoader::timerCallback(const ros::TimerEvent &e) {
  // publish stop sign markers
  stoplines_msg_->markers.resize(stoplines_.size());
  for (int i = 0; i < stoplines_.size(); i++) {
    std::vector<std::vector<double>> stopline = stoplines_[i];
    stoplines_msg_->markers[i] = constructstoplineMarker(stopline, i);
  }
  if (stoplines_.size() > 0) {
    stoplines_marker_pub_.publish(stoplines_msg_);
  }

  // publish crosswalk markers
  crosswalks_msg_->markers.resize(crosswalks_.size());
  for (int i = 0; i < crosswalks_.size(); i++) {
    std::vector<std::vector<double>> stopline = crosswalks_[i];
    crosswalks_msg_->markers[i] = constructCrosswalkMarker(stopline, i);
  }
  if (crosswalks_.size() > 0) {
    crosswalks_marker_pub_.publish(crosswalks_msg_);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traffic_info_loader");
  ros::NodeHandle nh("~");

  ////////////////////////////////
  // Map Loader instance
  ////////////////////////////////
  TrafficInfoLoader loader(nh);

  ////////////////////////////////
  // ROS Spin
  ////////////////////////////////
  ros::spin();

  return 0;
}
