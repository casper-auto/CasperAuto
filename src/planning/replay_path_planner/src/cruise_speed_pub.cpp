#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>

#include <ros/ros.h>
#include <std_msgs/Float32.h>

using namespace std;

double current_speed = 0;

void currentSpeedCallback(const std_msgs::Float32::ConstPtr& msg) {
  current_speed = msg->data;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cruise_speed_pub");
  ros::NodeHandle n("~");
  ros::Subscriber current_speed_sub = n.subscribe("/current_speed", 10, currentSpeedCallback);
  ros::Publisher cruise_speed_pub = n.advertise<std_msgs::Float32>("/cruise_speed", 10);
  ros::Rate rate(60);

  // prepare csv files
  ofstream out;

  while(ros::Time::now().toSec() <= 0.0001);

  vector<double> speed_cmds = {1, 3, 5, 3, 1};
  double time_interval = 10; // second
  double time_total = int(speed_cmds.size()) * time_interval;
  double time_elapsed = 0;
  double time_start = ros::Time::now().toSec();

  auto time_stamp = chrono::system_clock::now();
  time_t time_stamp_t = chrono::system_clock::to_time_t(time_stamp);
  string time_stamp_s = to_string(time_stamp_t);
  out.open("speed_control_" + time_stamp_s + ".csv");
  ROS_INFO("time_elapsed: %f, time_start: %f, time_total: %f", time_elapsed, time_start, time_total);

  while(ros::ok() && time_elapsed <= time_total) {
    ////////////////////////////////////////////////////////////////////////////
    // Compute Desired Cruise Speed
    ////////////////////////////////////////////////////////////////////////////
    double time_now = ros::Time::now().toSec();
    time_elapsed = time_now - time_start;
    int speed_index = time_elapsed / time_interval;
    speed_index = min(speed_index, int(speed_cmds.size()-1));
    double cruise_speed = speed_cmds[speed_index];
    out << time_elapsed << ", " << cruise_speed << ", " << current_speed << endl;
    ROS_INFO("Desired speed: %f, Actual speed: %f, Time Elapsed: %f", cruise_speed, current_speed, time_elapsed);

    ////////////////////////////////////////////////////////////////////////////
    // Publish Desired Cruise Speed
    ////////////////////////////////////////////////////////////////////////////
    std_msgs::Float32 msg;
    msg.data = cruise_speed;
    cruise_speed_pub.publish(msg);

    ////////////////////////////////////////////////////////////////////////////
    // ROS Spin
    ////////////////////////////////////////////////////////////////////////////
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("The iteration end.");
  }

  out.close();

  return 0;
}
