#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Generates a plan of waypoints to follow

It uses the current pose of the ego vehicle as starting point. If the
vehicle is respawned, the route is newly calculated.

The goal is either read from the ROS topic `/carla/<ROLE NAME>/move_base_simple/goal`, if available
(e.g. published by RVIZ via '2D Nav Goal') or a fixed spawnpoint is used.

The calculated route is published on '/carla/<ROLE NAME>/waypoints'
"""
import math
import threading
import random

import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int32
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point
from carla_msgs.msg import CarlaWorldInfo
from traffic_msgs.msg import CenterLanes

import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from utils.frenet_utils import *

LaneInfo = namedtuple('LaneInfo', ['start_wp', 'target_wp', 'merging_wp'])

target_lane_val = 100
ad_lane_val = 0

class CarlaToRosWaypointConverter(object):

    """
    This class generates a plan of waypoints to follow.

    The calculation is done whenever:
    - the ego_vehicle vehicle appears
    - a new goal is set
    """
    def __init__(self, carla_world):

        self.world = carla_world
        self.map = carla_world.get_map()
        self.ego_vehicle = None
        self.role_name = rospy.get_param("~role_name", 'ego_vehicle')
        self.center_lines_publisher = rospy.Publisher(
            '/region/lanes_center', CenterLanes, queue_size=1, latch=True)
        self.target_lane_publisher = rospy.Publisher(
            '/region/target_lane_id', Int32, queue_size=1, latch=True)

        self.lane_info_dict = {}

        # GOAL POINT FOR TARGET LANE ID - MAYBE MOVE TO ROUTE PLANNER
        self.target_lane_key = None
        goal_pt_param = rospy.get_param("~goal_point")
        if goal_pt_param:
            goal_point = goal_pt_param.split(',')
            if len(goal_point) != 3:
                raise ValueError("Invalid goal point '{}'".format(goal_pt_param))
            else:
                self.target_pt_lane = [float(goal_point[0]), float(goal_point[1]), float(goal_point[2])]

        # SCENARIO PARAMATERS
        lanes_ids = rospy.get_param('/scenario/lanes')
        if not lanes_ids:
            NameError("Missing lane information")
            exit()

        self.scenario_type = rospy.get_param('/scenario/type_scenario')
        if not self.scenario_type:
            NameError("Missing lane information")
            exit()
        if self.scenario_type == "cross-traffic":
            if not target_lane_val in lanes_ids:
                raise ValueError("Cross traffic needs a target [100] lane")
                exit()
            if not ad_lane_val in lanes_ids:
                raise ValueError("Cross traffic needs a start ad [0] lane")
                exit()

        elif self.scenario_type == "same-direction-traffic":
            if not ad_lane_val in lanes_ids:
                raise ValueError("Same traffic needs at leaste a lane [0]")
                exit()
        else:
            raise ValueError("Unknown type of scenario")
            exit()

        for lane_vals in lanes_ids:

            key_st_pt = "scenario/start_point_lane" + str(lane_vals)
            key_tg_pt = "scenario/target_point_lane" + str(lane_vals)
            key_mg_pt = "scenario/merging_point_lane" + str(lane_vals)

            start_pt = rospy.get_param(key_st_pt)
            if not start_pt:
                NameError("Missing lane '{}' start point".format(lane_vals))
                exit()
            start_wp = self.map.get_waypoint(carla.Location(start_pt[0], -start_pt[1], start_pt[2]))

            target_pt = rospy.get_param(key_tg_pt)
            if not target_pt:
                NameError("Missing lane '{}' target point".format(lane_vals))
                exit()
            target_wp = self.map.get_waypoint(carla.Location(target_pt[0], -target_pt[1], target_pt[2]))

            merge_pt = rospy.get_param(key_mg_pt)
            if not merge_pt:
                NameError("Missing lane '{}' merging point".format(lane_vals))
                exit()
            merge_wp = self.map.get_waypoint(carla.Location(merge_pt[0], -merge_pt[1], merge_pt[2]))

            self.lane_info_dict[lane_vals] = LaneInfo(start_wp=start_wp, target_wp=target_wp, merging_wp=merge_wp)

            if lane_vals == target_lane_val:
                self.merging_wp_target = merge_wp
                self.merging_s_set = False


        # AD position
        self.ego_odom_subscriber = rospy.Subscriber("/carla/{}/odometry".format(self.role_name), Odometry, self.on_odom, queue_size=10)
        self.ad_current_pose = None
        self.ad_current_lane = -100
        self.ad_past_lane = -100
        self.passed_merging = False


        self._update_lock = threading.Lock()

        self.lanes_published = False
        self.lane_fixed = False
        self.lanes_dict = {}
        self.lanes_frenet = {}

        # use callback to wait for ego vehicle
        rospy.loginfo("Waiting for ego vehicle...")
        self.world.on_tick(self.find_ego_vehicle_actor)

    def reset(self):
        self.lanes_published = False
        self.lane_fixed = False
        self.lanes_dict = {}
        self.lanes_frenet = {}
        self.ad_current_pose = None
        self.ad_current_lane = -100
        self.ad_past_lane = -100
        self.passed_merging = False

    def on_odom(self, odom):

        if not self.lane_fixed:
            self.lane_fixed = True
            self.reroute()

        # Figure out ad car lane
        if self.lanes_dict:
            if self.scenario_type == "cross-traffic":
                # Set the s for merging if not done yet
                if not self.merging_s_set:
                    frenet = self.lanes_frenet[ad_lane_val]
                    s_mg, d_mg, conv_mg = get_frenet(self.merging_wp_target.transform.location.x, -self.merging_wp_target.transform.location.y, frenet[0], frenet[1])
                    self.merging_s_set = True
                    self.merging_s = s_mg

                # We check if he have moved passed the merging point
                frenet = self.lanes_frenet[ad_lane_val]
                s_ad, d_ad, conv_ad = get_frenet(odom.pose.pose.position.x, odom.pose.pose.position.y, frenet[0], frenet[1])

                # Check for reset
                if self.passed_merging and s_ad < self.merging_s:
                    self.reset()
                    return

                if s_ad >= self.merging_s:
                   self.ad_current_lane = target_lane_val
                   self.passed_merging = True

            elif self.scenario_type == "same-direction-traffic":
                # We check for lateral distance to the center line and the AD vehicle are
                self.ad_current_lane = -100
                min_val = 1e15
                for key, value in self.lanes_frenet.items():
                    s, d, conv = get_frenet(odom.pose.pose.position.x, odom.pose.pose.position.y, value[0], value[1])

                    if abs(d) < min_val:
                        min_val = abs(d)
                        self.ad_current_lane = key
            else:
                raise ValueError("Unknown type of scenario")
                exit()

            if self.ad_current_lane != self.ad_past_lane:
                self.ad_past_lane = self.ad_current_lane
                self.lanes_published = False

        if not self.lanes_published:
            self.publish_all_lanes()

    def reroute(self):
        """
        Triggers a rerouting
        """
        if self.ego_vehicle is None:
            # no ego vehicle, remove route if published
            self.lanes_dict = {}
        else:
            self.calculate_route()

    def find_ego_vehicle_actor(self, _):
        """
        Look for an carla actor with name 'ego_vehicle'
        """
        with self._update_lock:
            ego_vehicle = None
            for actor in self.world.get_actors():
                if actor.attributes.get('role_name') == self.role_name:
                    ego_vehicle = actor
                    break

            ego_vehicle_changed = False
            if ego_vehicle is None and self.ego_vehicle is not None:
                ego_vehicle_changed = True

            if not ego_vehicle_changed and ego_vehicle is not None and self.ego_vehicle is None:
                ego_vehicle_changed = True

            if not ego_vehicle_changed and ego_vehicle is not None and \
                    self.ego_vehicle is not None and ego_vehicle.id != self.ego_vehicle.id:
                ego_vehicle_changed = True

            if ego_vehicle_changed:
                rospy.loginfo("Ego vehicle changed.")
                self.ego_vehicle = ego_vehicle
                self.reroute()

    def calculate_route(self):

        """
        Calculate the source lane line
        """
        dao = GlobalRoutePlannerDAO(self.map, sampling_resolution=1.0)
        grp = GlobalRoutePlanner(dao)
        grp.setup()

        self.lanes_dict = {}
        self.lanes_frenet = {}
        self.target_lane_key = None

        # For each lane find the route and save it in the dictionary
        for key, value in self.lane_info_dict.items():

            """
            Find the center lane for each of the lanes
            """

            lane_route = grp.trace_route(value.start_wp.transform.location, value.target_wp.transform.location)
            lane_route.append([value.target_wp, None])

            print("Lane ID: %d source: %d, Length target: %d" % (key, len(lane_route), len(lane_route)))

            #Transform to path
            path_route = Path()
            prev_wp = None
            for wp in lane_route:
                if prev_wp is not None:
                    dist_wp = distance(wp[0].transform.location.x, wp[0].transform.location.y, prev_wp[0].transform.location.x, prev_wp[0].transform.location.y)
                    if dist_wp < 0.1:
                        continue

                prev_wp = wp

                pose = PoseStamped()
                pose.pose.position.x = wp[0].transform.location.x
                pose.pose.position.y = -wp[0].transform.location.y
                pose.pose.position.z = wp[0].transform.location.z

                quaternion = tf.transformations.quaternion_from_euler(
                    0, 0, -math.radians(wp[0].transform.rotation.yaw))
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                path_route.poses.append(pose)

            self.lanes_dict[key] = path_route
            lane_line_list, lane_s_map = path_to_list(path_route)
            self.lanes_frenet[key] = [lane_line_list, lane_s_map]

            # Find in which lane the global target point is and save the key
            s, d, conv = get_frenet(self.target_pt_lane[0], -self.target_pt_lane[1], lane_line_list, lane_s_map)
            if abs(d) < value.target_wp.lane_width / 2:
                self.target_lane_key = key
                print("Target lane in global id: %d" % key)

    def publish_all_lanes(self):
        """
        Publish the ROS message containing the waypoints
        """
        msg_center_lanes = CenterLanes()
        msg_center_lanes.header.frame_id = "map"
        # msg.header.stamp = rospy.Time.now()
        already_merged = False
        if self.lanes_dict:
            for key, value in self.lane_info_dict.items():
                # ID
                if self.scenario_type == "cross-traffic":
                    lane_val_set = False
                    if self.ad_current_lane == target_lane_val:
                        if key == ad_lane_val:
                            # We do not need this lane anymore
                            already_merged = True
                            continue

                        if key == target_lane_val:
                            msg_center_lanes.ids.append(ad_lane_val)
                            lane_val_set = True

                    if not lane_val_set:
                        msg_center_lanes.ids.append(key)


                elif self.scenario_type == "same-direction-traffic":
                    msg_center_lanes.ids.append(key - self.ad_current_lane)
                else:
                    raise ValueError("Unknown type of scenario")
                    exit()

                # Center lines
                msg_center_lanes.center_lines.append(self.lanes_dict[key])

                #Width
                msg_center_lanes.lanes_width.append(value.target_wp.lane_width)

                #Merging point
                pt_merging = Point()
                pt_merging.x = value.merging_wp.transform.location.x
                pt_merging.y = -value.merging_wp.transform.location.y
                pt_merging.z = value.merging_wp.transform.location.z
                msg_center_lanes.crossing_ad_points.append(pt_merging)

            self.lanes_published = True

        self.center_lines_publisher.publish(msg_center_lanes)

        # Target lane
        if self.target_lane_key is not None:
            if self.scenario_type == "cross-traffic":
                if already_merged:
                    self.target_lane_publisher.publish(ad_lane_val)
                else:
                    self.target_lane_publisher.publish(target_lane_val)
            elif self.scenario_type == "same-direction-traffic":
                self.target_lane_publisher.publish(self.target_lane_key - self.ad_current_lane)
            else:
                raise ValueError("Unknown type of scenario")
                exit()

def main():
    """
    main function
    """
    rospy.init_node("lane_line_publisher", anonymous=True)

    # wait for ros-bridge to set up CARLA world
    rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
    try:
        rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
    except rospy.ROSException as e:
        rospy.logerr("Timeout while waiting for world info!")
        raise e

    host = rospy.get_param("/carla/host", "127.0.0.1")
    port = rospy.get_param("/carla/port", 2000)

    rospy.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(
        host=host, port=port))

    try:
        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(2)

        carla_world = carla_client.get_world()

        rospy.loginfo("Connected to Carla.")

        waypointConverter = CarlaToRosWaypointConverter(carla_world)

        rospy.spin()
        del waypointConverter
        del carla_world
        del carla_client

    finally:
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
