#!/usr/bin/env python

"""
Design:

Imagine in a lane change scenario, what information are essential to make a
reasonalble decision about when and how to complete the lane change action.

ego_vehicle: x, y, yaw, vel

other_vehicles:
    vehicles_left:
        A list of vehicles in the left lane if a left lane exists which could be
        empty or null.
    vehicles_right:
        A list of vehicles in the right lane if a left lane exists which could be
        empty or null.
    vehicle_ahead:
        A single vehicle that is ahead of the ego vehicle within a certain range
    vehicle_behind:
        A single vehicle that is behind the ego vehicle within a certain range.
"""

from __future__ import print_function

import datetime
import math
import numpy as np
from operator import itemgetter, attrgetter
import rospy
import tf
from std_msgs.msg import Float32
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from derived_object_msgs.msg import Object
from derived_object_msgs.msg import ObjectArray
from carla_msgs.msg import CarlaWorldInfo
from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaCollisionEvent
from traffic_msgs.msg import VehicleState
from traffic_msgs.msg import VehicleStateArray

from utils.frenet_utils import *
from utils.noise_utils import *


class RegionOfInterest(object):

    """
    The class will maintain a system of the local traffic which holds the
    information for the ego vehicle to make the decision in the lane change
    process.
    """

    def __init__(self, role_name, debug=False):
        """
        Initialize the ego_vehicle, other vehicles and other elements.
        """
        self.role_name = role_name
        self.detect_range = rospy.get_param("~detect_range", 50.0)
        self.debug = debug

        # initialize placeholders for raw data
        self.objects = []
        self.source_lane_line = Path()
        self.target_lane_line = Path()

        # main status info to be processed
        self.ego_state = VehicleState()
        self.ego_state.width = rospy.get_param("~car_width", 2.0)
        self.ego_state.length = rospy.get_param("~car_length", 5.0)

        self.ego_perception = VehicleStateArray()
        self.source_lane = VehicleStateArray()
        self.target_lane = VehicleStateArray()
        self.obstacle_ahead = None

        self.source_lane_line_list = []
        self.target_lane_line_list = []
        self.source_lane_s_map = []
        self.target_lane_s_map = []

        self.tf_listener = tf.TransformListener()

        # subscribers
        self.vehicle_status_subscriber = rospy.Subscriber(
            "/carla/{}/vehicle_status".format(self.role_name), CarlaEgoVehicleStatus, self.vehicle_status_updated, queue_size=10)
        self.ego_odom_subscriber = rospy.Subscriber(
            "/carla/{}/odometry".format(self.role_name), Odometry, self.ego_odom_updated, queue_size=10)
        self.objects_subscriber = rospy.Subscriber(
            "/carla/{}/objects".format(self.role_name), ObjectArray, self.objects_updated, queue_size=10)

        self.source_lane_line_subscriber = rospy.Subscriber(
            "/region/source_lane/center_line", Path, self.source_lane_line_updated)
        self.target_lane_line_subscriber = rospy.Subscriber(
            "/region/target_lane/center_line", Path, self.target_lane_line_updated)

        # publishers
        self.ego_state_pub = rospy.Publisher('/region/ego_state', VehicleState, queue_size=10)
        self.ego_perception_pub = rospy.Publisher('/region/ego_vehicle/vehicles', VehicleStateArray, queue_size=10)
        self.source_lane_pub = rospy.Publisher('/region/source_lane/vehicles', VehicleStateArray, queue_size=10)
        self.target_lane_pub = rospy.Publisher('/region/target_lane/vehicles', VehicleStateArray, queue_size=10)
        self.obstacle_ahead_pub = rospy.Publisher('/region/obstacle_ahead', VehicleState, queue_size=10)
        self.lane_line_distance_pub = rospy.Publisher('/region/lane_line_distance', Float32, queue_size=10)

        # noise values
        self._error_struct = ErrorPerceptionLocalizationStruct()

        self._error_struct._frequency_update = rospy.get_param("~frequency_update", 10.0)
        self._add_perception_errors = rospy.get_param("~add_perception_errors", True)
        self._add_localization_errors = rospy.get_param("~add_localization_errors", True)

        self._error_struct._perception_latency_detection = rospy.get_param("~perception_latency_detection", 1.0)
        self._error_struct._detection_alpha = rospy.get_param("~detection_alpha", 2.0)
        self._error_struct._detection_beta = rospy.get_param("~detection_beta", 5.0)
        self._error_struct._perception_latency = rospy.get_param("~perception_latency", 0.15)
        self._error_struct._tracking_error_prob = rospy.get_param("~tracking_error_prob", 0.05)
        self._error_struct._max_frames_not_detected = rospy.get_param("~max_frames_not_detected", 4)
        self._error_struct._prob_pcp_buildup_start = rospy.get_param("~prob_pcp_buildup_start", 0.05)
        self._error_struct._size_pcp_jump = rospy.get_param("~size_pcp_jump", 0.75)
        self._error_struct._seconds_pcp_acum = rospy.get_param("~seconds_pcp_acum", 2.0)
        self._error_struct._noise_agents_pos_lat_start = rospy.get_param("~noise_agents_pos_lat_start", 0.1)
        self._error_struct._noise_agents_pos_long_start = rospy.get_param("~noise_agents_pos_long_start", 0.1)
        self._error_struct._noise_agents_head_start = rospy.get_param("~noise_agents_head_start", 10.0)
        self._error_struct._noise_agents_vel_start = rospy.get_param("~noise_agents_vel_start", 0.05)
        self._error_struct._noise_agents_acc_start = rospy.get_param("~noise_agents_acc_start", 0.05)
        self._error_struct._noise_agents_pos_lat_rel = rospy.get_param("~noise_agents_pos_lat_rel", 0.05)
        self._error_struct._noise_agents_pos_long_rel = rospy.get_param("~noise_agents_pos_long_rel", 0.075)
        self._error_struct._noise_agents_head_rel = rospy.get_param("~noise_agents_head_rel", 0.25)
        self._error_struct._noise_agents_vel_rel = rospy.get_param("~noise_agents_vel_rel", 0.1)
        self._error_struct._noise_agents_acc_rel = rospy.get_param("~noise_agents_acc_rel", 0.1)
        self._error_struct._thld_agents_stopped = rospy.get_param("~thld_agents_stopped", 0.1)
        self._error_struct._prob_agents_not_stopped = rospy.get_param("~prob_agents_not_stopped", 0.05)

        self._error_struct._latency_localization = rospy.get_param("~latency_localization", 0.15)
        self._error_struct._prob_loc_buildup_start = rospy.get_param("~prob_loc_buildup_start", 0.05)
        self._error_struct._prob_loc_small_jump = rospy.get_param("~prob_loc_small_jump", 0.8)
        self._error_struct._size_loc_small_jump = rospy.get_param("~size_loc_small_jump", 0.2)
        self._error_struct._size_loc_big_jump = rospy.get_param("~size_loc_big_jump", 0.75)
        self._error_struct._seconds_loc_acum = rospy.get_param("~seconds_loc_acum", 2.0)
        self._error_struct._noise_loc_pos_lat = rospy.get_param("~noise_loc_pos_lat", 0.1)
        self._error_struct._noise_loc_pos_long = rospy.get_param("~noise_loc_pos_long", 0.3)
        self._error_struct._noise_loc_head = rospy.get_param("~noise_loc_head", 5.0)
        self._error_struct._noise_loc_speed = rospy.get_param("~noise_loc_speed", 0.2)

        self._error_localization = ErrorLocalization(self._error_struct)
        self._dictionary_objects = {}

    def __del__(self):
        self.vehicle_status_subscriber.unregister()
        self.ego_odom_subscriber.unregister()
        self.objects_subscriber.unregister()
        self.collision_subscriber.unregister()

    def vehicle_status_updated(self, vehicle_status):
        """
        Callback on vehicle status updates
        """
        self.ego_state.accel.accel = vehicle_status.acceleration

    def ego_odom_updated(self, odom):
        """
        Callback on vehicle status updates
        """
        self.ego_state.pose.pose = odom.pose.pose
        self.ego_state.twist.twist = odom.twist.twist

        # use source lane center line as frenet_d = 0
        if len(self.source_lane_line_list) != 0:
            ego_x = self.ego_state.pose.pose.position.x
            ego_y = self.ego_state.pose.pose.position.y
            quaternion = self.ego_state.pose.pose.orientation
            quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
            # yaw = -math.degrees(yaw)
            ego_s, ego_d, conv = get_frenet(ego_x, ego_y, self.source_lane_line_list, self.source_lane_s_map)
            if not conv:
                return

            self.ego_state.s = ego_s
            self.ego_state.d = ego_d

            if self._add_localization_errors:
                # Compute error localization values and substitute them before sending
                s_new, d_new, v_new, yaw_new = self._error_localization.LocalizationUpdate(ego_s, ego_d, odom.twist.twist.linear.x, yaw)

                x_new, y_new = get_xy(s_new, d_new, self.source_lane_line_list, self.source_lane_s_map)

                # print("Original")
                # print("X: %f, Y: %f, V: %f, Yaw: %f, s: %f, d: %f" % (ego_x, ego_y, odom.twist.twist.linear.x, yaw, ego_s, ego_d))

                # print("Modified")
                # print("X: %f, Y: %f, V: %f, Yaw: %f, s: %f, d: %f" % (x_new, y_new, v_new, yaw_new, s_new, d_new))

                self.ego_state.pose.pose.position.x = x_new
                self.ego_state.pose.pose.position.y = y_new
                self.ego_state.twist.twist.linear.x = v_new
                self.ego_state.s = s_new
                self.ego_state.d = d_new
                q_new = tf.transformations.quaternion_from_euler(roll, pitch, yaw_new)
                self.ego_state.pose.pose.orientation.x = q_new[0]
                self.ego_state.pose.pose.orientation.y = q_new[1]
                self.ego_state.pose.pose.orientation.z = q_new[2]
                self.ego_state.pose.pose.orientation.w = q_new[3]

            self.ego_state_pub.publish(self.ego_state)

    def objects_updated(self, object_array):
        """
        Callback on gnss position updates
        """
        self.objects = object_array.objects

        ###############################################
        # no lane information
        ###############################################
        if len(self.source_lane_line_list) == 0 or len(self.target_lane_line_list) == 0:
            return

        ###############################################
        # got lane information
        ###############################################
        del self.ego_perception.vehicles[:]
        del self.source_lane.vehicles[:]
        del self.target_lane.vehicles[:]

        # In source lane, only need to put in a leader vehiche and
        # a follower vehicle
        source_leader = VehicleState()
        source_follower = VehicleState()

        for i in xrange(len(self.objects)):
            obj = self.objects[i]
            obj_x = float(obj.pose.position.x)
            obj_y = float(obj.pose.position.y)

            ego_x = self.ego_state.pose.pose.position.x
            ego_y = self.ego_state.pose.pose.position.y

            distance_to_ego = distance(ego_x, ego_y, obj_x, obj_y)

            if distance_to_ego > self.detect_range:
                continue

            # filter vehicles in the source lane
            obj_s, obj_d, obj_conv = get_frenet(obj_x, obj_y, self.source_lane_line_list, self.source_lane_s_map)

            if obj_conv and abs(obj_d) <= 0.5:
                veh = VehicleState()
                veh.header = obj.header
                veh.lifetime_id = obj.id
                veh.pose.pose = obj.pose
                veh.twist.twist = obj.twist
                veh.accel.accel = obj.accel
                veh.length = obj.shape.dimensions[0]
                veh.width = obj.shape.dimensions[1]

                if len(self.source_lane_line_list) != 0:
                    veh_x = veh.pose.pose.position.x
                    veh_y = veh.pose.pose.position.y
                    quaternion = veh.pose.pose.orientation
                    quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
                    _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
                    # yaw = -math.degrees(yaw)

                    veh.s = obj_s
                    veh.d = obj_d

                    if obj_s > self.ego_state.s:
                        if source_leader.local_id == 0 or obj_s < source_leader.s:
                            veh.local_id = 11 # 1-: source lane; -1: 1st car as leader
                            source_leader = veh
                    else:
                        if source_follower.local_id == 0 or obj_s > source_follower.s:
                            veh.local_id = 12 # 1-:source lane; -2: 2nd car as follower
                            source_follower = veh

            # filter vehicles in the target lane
            obj_s, obj_d, obj_conv = get_frenet(obj_x, obj_y, self.target_lane_line_list, self.target_lane_s_map)

            if obj_conv and abs(obj_d) <= 0.5:
                veh = VehicleState()
                veh.header = obj.header
                veh.lifetime_id = obj.id
                veh.pose.pose = obj.pose
                veh.twist.twist = obj.twist
                veh.accel.accel = obj.accel
                veh.length = obj.shape.dimensions[0]
                veh.width = obj.shape.dimensions[1]

                send_car = False

                if len(self.target_lane_line_list) != 0:
                    veh_x = veh.pose.pose.position.x
                    veh_y = veh.pose.pose.position.y
                    quaternion = veh.pose.pose.orientation
                    quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
                    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
                    # yaw = -math.degrees(yaw)

                    veh.s, veh.d, obj_conv = get_frenet(veh_x, veh_y, self.source_lane_line_list, self.source_lane_s_map)

                if self._add_perception_errors:
                    new_veh, send_car = self.check_and_add_noise(veh, self.target_lane_line_list)

                    if send_car:
                        self.ego_perception.vehicles.append(new_veh)
                        self.target_lane.vehicles.append(new_veh)
                else:
                    self.ego_perception.vehicles.append(veh)
                    self.target_lane.vehicles.append(veh)

        ###############################################
        # sort vehicle by s
        ###############################################
        if self._add_perception_errors:
            if source_leader.local_id != 0:
                new_veh, send_car = self.check_and_add_noise(source_leader, self.source_lane_line_list)

                if send_car:
                    self.ego_perception.vehicles.append(new_veh)
                    self.source_lane.vehicles.append(new_veh)


            if source_follower.local_id != 0:
                new_veh, send_car = self.check_and_add_noise(source_follower, self.source_lane_line_list)

                if send_car:
                    self.ego_perception.vehicles.append(new_veh)
                    self.source_lane.vehicles.append(new_veh)

        else:
            self.ego_perception.vehicles.append(source_leader)
            self.ego_perception.vehicles.append(source_follower)
            self.source_lane.vehicles.append(source_leader)
            self.source_lane.vehicles.append(source_follower)

        self.target_lane.vehicles = sorted(self.target_lane.vehicles, key=attrgetter('s'), reverse=True)
        for i in xrange(len(self.target_lane.vehicles)):
          self.target_lane.vehicles[i].local_id = 21 + i # 2-: target lane; -i: ordered by s

        ###############################################
        # obstacle ahead exits if source leader is static
        ###############################################
        if source_leader.local_id != 0 and source_leader.twist.twist.linear.x < 0.01:
            self.obstacle_ahead = source_leader
        else:
            self.obstacle_ahead = None

        # rospy.loginfo('{} object in all'.format(len(self.objects)))
        # rospy.loginfo('{} cars in the source lane'.format(len(self.source_lane.vehicles)))
        # rospy.loginfo('{} cars in the target lane'.format(len(self.target_lane.vehicles)))

        self.ego_perception_pub.publish(self.ego_perception)
        self.source_lane_pub.publish(self.source_lane)
        self.target_lane_pub.publish(self.target_lane)
        if self.obstacle_ahead:
            self.obstacle_ahead_pub.publish(self.obstacle_ahead)

    def check_and_add_noise(self, veh, line_list):

        class_aux = self._dictionary_objects.get(veh.lifetime_id)

        t_now = rospy.get_time()

        quaternion = veh.pose.pose.orientation
        quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

        dist_with_ad = math.sqrt((self.ego_state.pose.pose.position.x - veh.pose.pose.position.x) ** 2 + (self.ego_state.pose.pose.position.y - veh.pose.pose.position.y) ** 2)

        if class_aux == None:
            #Create new class
            class_aux = ErrorPerception(self._error_struct, t_now)

            s_new, d_new, v_new, a_new, yaw_new, send_car =  class_aux.PerceptionUpdate(veh.s, veh.d, veh.twist.twist.linear.x, veh.accel.accel.linear.x, yaw, t_now, dist_with_ad)

            self._dictionary_objects[veh.lifetime_id] = class_aux
        else:
            #Update class. Remove if too much time has passed and create a new one
            if t_now - class_aux._prev_time > 5.0:
                del self._dictionary_objects[veh.lifetime_id]

                class_aux = ErrorPerception(self._error_struct, t_now)

                s_new, d_new, v_new, a_new, yaw_new, send_car =  class_aux.PerceptionUpdate(veh.s, veh.d, veh.twist.twist.linear.x, veh.accel.accel.linear.x, yaw, t_now, dist_with_ad)

                self._dictionary_objects[veh.lifetime_id] = class_aux
            else:
                s_new, d_new, v_new, a_new, yaw_new, send_car =  class_aux.PerceptionUpdate(veh.s, veh.d, veh.twist.twist.linear.x, veh.accel.accel.linear.x, yaw, t_now, dist_with_ad)

        if send_car:
            s_map = get_s_map(line_list)
            x_new, y_new = get_xy(s_new, d_new, s_map, line_list)

            # print("Car ID: %d" % veh.lifetime_id)
            # print("Original")
            # print("X: %f, Y: %f, V: %f, Acc: %f, Yaw: %f, s: %f, d: %f" % (veh.pose.pose.position.x, veh.pose.pose.position.y, veh.twist.twist.linear.x, veh.accel.accel.linear.x, yaw, veh.s, veh.d))

            # print("Modified")
            # print("X: %f, Y: %f, V: %f, Acc: %f, Yaw: %f, s: %f, d: %f" % (x_new, y_new, v_new, a_new, yaw_new, s_new, d_new))
            # print("")

            # Overwrite variables
            veh.pose.pose.position.x = x_new
            veh.pose.pose.position.y = y_new
            veh.twist.twist.linear.x = v_new
            veh.accel.accel.linear.x = a_new
            q_new = tf.transformations.quaternion_from_euler(roll, pitch, yaw_new)
            veh.pose.pose.orientation.x = q_new[0]
            veh.pose.pose.orientation.y = q_new[1]
            veh.pose.pose.orientation.z = q_new[2]
            veh.pose.pose.orientation.w = q_new[3]

        # else:
        #     print("Invalid car")
        #     ("")

        return veh, send_car

    def source_lane_line_updated(self, path):
        """
        Callback on gnss position updates
        """
        self.source_lane_line = path
        self.source_lane_line_list, self.source_lane_s_map = path_to_list(self.source_lane_line)

    def target_lane_line_updated(self, path):
        """
        Callback on gnss position updates
        """
        self.target_lane_line = path
        self.target_lane_line_list, self.target_lane_s_map = path_to_list(self.target_lane_line)

    def display_status(self):
        """
        render the display
        """
        print('********** Display **********')

        ###############################################
        # ego state display
        ###############################################
        ego_x = self.ego_state.pose.pose.position.x
        ego_y = self.ego_state.pose.pose.position.y
        quaternion = self.ego_state.pose.pose.orientation
        quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        # yaw = -math.degrees(yaw)
        ego_s = self.ego_state.s
        ego_d = self.ego_state.d

        print('ego_vehicle:')
        print('  x: {0:.2f}, y: {1:.2f}, yaw: {2:.2f}, vel: {3:.2f}, accel: {4:.2f}'.format(
            ego_x,
            ego_y,
            yaw,
            self.ego_state.twist.twist.linear.x,
            self.ego_state.accel.accel.linear.x))
        print('  s: {0:.2f}, d: {1:.2f}'.format(ego_s, ego_d))

        ###############################################
        # other vehicles state display
        ###############################################
        print('other vehicles nearby:')
        print('  Number: {}'.format(len(self.objects)))
        for i in xrange(len(self.objects)):
            object = self.objects[i]
            veh_x = object.pose.position.x
            veh_y = object.pose.position.y
            quaternion = object.pose.orientation
            quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
            # yaw = -math.degrees(yaw)
            veh_s, veh_d, obj_conv = get_frenet(veh_x, veh_y, self.source_lane_line_list, self.source_lane_s_map)

            if math.sqrt(math.pow(ego_x - veh_x, 2) + math.pow(ego_y - veh_y, 2)) > 50:
                continue

            print('  Veh_{}:'.format(i) + \
                '  x: {0:.2f}, y: {1:.2f}, yaw: {2:.2f}, vel: {3:.2f}, accel: {4:.2f}'.format(
                veh_x,
                veh_y,
                yaw,
                object.twist.linear.x,
                object.accel.linear.x))
            print('  s: {0:.2f}, d: {1:.2f}'.format(veh_s, veh_d))

    def run(self):
        ###############################################
        # lane_line_distance
        ###############################################
        if len(self.source_lane_line_list) < 2 or len(self.target_lane_line_list) < 2:
            lane_line_distance = 0
        else:
            lane_line_distance = dist_to_line(self.target_lane_line_list[0],
                                              self.target_lane_line_list[1],
                                              self.source_lane_line_list[0])

        self.lane_line_distance_pub.publish(lane_line_distance)

        # print("subscribers: " , self.target_lane_pub.get_num_connections())


def main():
    """
    main function
    """
    rospy.init_node('region_of_interest_node', anonymous=True)

    # wait for ros-bridge to set up CARLA world
    rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
    try:
        rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
    except rospy.ROSException as e:
        rospy.logerr("Timeout while waiting for world info!")
        raise e

    rospy.loginfo("Connected to Carla.")

    region = RegionOfInterest(role_name='ego_vehicle', debug=True)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        region.run()
        rate.sleep()


if __name__ == '__main__':

    main()
