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
from std_msgs.msg import Float32
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped
from carla_msgs.msg import CarlaWorldInfo

import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO


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
        self.source_lane_line_publisher = rospy.Publisher(
            '/region/source_lane/center_line', Path, queue_size=10, latch=True)
        self.target_lane_line_publisher = rospy.Publisher(
            '/region/target_lane/center_line', Path, queue_size=10, latch=True)
        self.source_lane_width_publisher = rospy.Publisher(
            '/region/source_lane/lane_width', Float32, queue_size=10, latch=True)
        self.target_lane_width_publisher = rospy.Publisher(
            '/region/target_lane/lane_width', Float32, queue_size=10, latch=True)

        self.ad_start_wp = None
        # check argument and set spawn_point
        ad_start_point_param = rospy.get_param('~spawn_point')
        if ad_start_point_param:
            rospy.loginfo("Using ros parameter for spawnpoint: {}".format(ad_start_point_param))
            ad_start_point = ad_start_point_param.split(',')
            if len(ad_start_point) != 6:
                raise ValueError("Invalid spawnpoint '{}'".format(ad_start_point_param))
            self.ad_start_wp = self.map.get_waypoint(carla.Location(float(ad_start_point[0]), float(ad_start_point[1]), float(ad_start_point[2])))
            

        self.agent_start_wp = None
        # check argument and set agent_point
        agent_start_point_param = rospy.get_param('~agent_point')
        if agent_start_point_param:
            rospy.loginfo("Using ros parameter for agent location: {}".format(agent_start_point_param))
            agent_start_point = agent_start_point_param.split(',')
            if len(agent_start_point) != 3:
                raise ValueError("Invalid agent start point '{}'".format(agent_start_point_param))
            self.agent_start_wp = self.map.get_waypoint(carla.Location(float(agent_start_point[0]), float(agent_start_point[1]), float(agent_start_point[2])))

        self.target_point_wp = None
        # check argument and set agent_point
        target_point_param = rospy.get_param('~target_point')
        if target_point_param:
            rospy.loginfo("Using ros parameter for target location: {}".format(target_point_param))
            target_point = target_point_param.split(',')
            if len(target_point) != 3:
                raise ValueError("Invalid target point '{}'".format(target_point_param))
            self.target_point_wp = self.map.get_waypoint(carla.Location(float(target_point[0]), float(target_point[1]), float(target_point[2])))

        self.source_lane_width = 3.5
        self.target_lane_width = 3.5

        self.source_lane_route = None
        self.target_lane_route = None

        self.lane_fixed = False
        self.target_published = False
        self.source_published = False

        self.ego_odom_subscriber = rospy.Subscriber(
            "/carla/{}/odometry".format(self.role_name), Odometry, self.on_odom, queue_size=10)

        self._update_lock = threading.Lock()

        # use callback to wait for ego vehicle
        rospy.loginfo("Waiting for ego vehicle...")
        self.world.on_tick(self.find_ego_vehicle_actor)

    def on_odom(self, odom):
        # rospy.loginfo("Odometry: x={}, y={}, z={}".format(
        #               odom.pose.pose.position.x,
        #               odom.pose.pose.position.y,
        #               odom.pose.pose.position.z))

        pose = odom.pose.pose

        if not self.ad_start_wp:
            ad_start_point = carla.Location()
            ad_start_point.x = pose.position.x
            ad_start_point.y = pose.position.y
            ad_start_point.z = pose.position.z
            self.ad_start_wp = self.map.get_waypoint(carla.Location(float(ad_start_point[0]), float(ad_start_point[1]), float(ad_start_point[2])))

        if not self.lane_fixed:
            self.lane_fixed = True
            self.reroute()

        if not self.source_published:
            self.publish_source_lane()

        if not self.target_published:
            self.publish_target_lane()

    def reroute(self):
        """
        Triggers a rerouting
        """
        if self.ego_vehicle is None or self.ad_start_wp is None:
            # no ego vehicle, remove route if published
            self.source_lane_route = None
            self.target_lane_route = None
        else:
            self.source_lane_route, self.target_lane_route = self.calculate_route()

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
        dao = GlobalRoutePlannerDAO(self.map, sampling_resolution=2.0)
        grp = GlobalRoutePlanner(dao)
        grp.setup()
        source_lane_route = grp.trace_route(self.ad_start_wp.transform.location, self.target_point_wp.transform.location)
        source_lane_route.append([self.target_point_wp, None])
        
        """
        Calculate the target lane line
        """
        
        target_lane_route = grp.trace_route(self.agent_start_wp.transform.location, self.target_point_wp.transform.location)
        target_lane_route.append([self.target_point_wp, None])

        print("Length source: %d, Length target: %d" % (len(source_lane_route), len(target_lane_route)))


        self.source_lane_width = self.ad_start_wp.lane_width
        self.target_lane_width = self.agent_start_wp.lane_width

        return source_lane_route, target_lane_route

    def publish_source_lane(self):
        """
        Publish the ROS message containing the waypoints
        """
        msg = Path()
        msg.header.frame_id = "map"
        # msg.header.stamp = rospy.Time.now()
        if self.source_lane_route is not None:
            for wp in self.source_lane_route:
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
                msg.poses.append(pose)

            self.source_published = True

        self.source_lane_line_publisher.publish(msg)
        self.source_lane_width_publisher.publish(self.source_lane_width)
        # rospy.loginfo("Published {} waypoints in the source lane route.".format(len(msg.poses)))

    def publish_target_lane(self):
        """
        Publish the ROS message containing the waypoints
        """
        msg = Path()
        msg.header.frame_id = "map"
        # msg.header.stamp = rospy.Time.now()
        if self.target_lane_route is not None:
            for wp in self.target_lane_route:
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
                msg.poses.append(pose)

            self.target_published = True

        self.target_lane_line_publisher.publish(msg)
        self.target_lane_width_publisher.publish(self.target_lane_width)
        # rospy.loginfo("Published {} waypoints in the target lane route.".format(len(msg.poses)))

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
