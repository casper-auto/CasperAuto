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
import sys
from operator import itemgetter, attrgetter
import rospy
import tf
import threading
from std_msgs.msg import Float32
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from derived_object_msgs.msg import Object
from derived_object_msgs.msg import ObjectArray
from carla_msgs.msg import CarlaWorldInfo
from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaCollisionEvent
from traffic_msgs.msg import PerceptionLanes
from traffic_msgs.msg import CenterLanes
from traffic_msgs.msg import PedestrianState, PedestrianStateArray
from traffic_msgs.msg import VehicleState, VehicleStateArray

from utils.frenet_utils import *
from utils.noise_utils import *

LaneInfo = namedtuple('LaneInfo', ['path', 'merging_wp', 'lane_width'])

target_lane_val = 100
ad_lane_val = 0

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

        self.mutex_var = threading.Lock()

        # initialize placeholders for raw data
        self.lanes_info_dict = {}
        self.lanes_frenet_dict = {}

        # Scenario information
        self.scenario_type = rospy.get_param('/scenario/type_scenario')
        if not self.scenario_type:
            NameError("Missing lane information")
            exit()
        self.merging_s_set = False

        # main status info to be processed
        self.ego_state = VehicleState()
        self.ego_state.width = rospy.get_param("~car_width", 2.0)
        self.ego_state.length = rospy.get_param("~car_length", 5.0)

        self.tf_listener = tf.TransformListener()

        # subscribers
        self.vehicle_status_subscriber = rospy.Subscriber(
            "/carla/{}/vehicle_status".format(self.role_name), CarlaEgoVehicleStatus, self.vehicle_status_updated, queue_size=10)
        self.ego_odom_subscriber = rospy.Subscriber(
            "/carla/{}/odometry".format(self.role_name), Odometry, self.ego_odom_updated, queue_size=10)
        self.objects_subscriber = rospy.Subscriber(
            "/carla/{}/objects".format(self.role_name), ObjectArray, self.objects_updated, queue_size=10)

        self.center_lanes_subscriber = rospy.Subscriber(
            "/region/lanes_center", CenterLanes, self.center_lanes_updated)

        # publishers
        self.ego_state_pub = rospy.Publisher('/region/ego_state', VehicleState, queue_size=10)
        self.object_array_pub = rospy.Publisher('/region/object_array', ObjectArray, queue_size=10)
        self.perception_lanes_pub = rospy.Publisher('/region/lanes_perception', PerceptionLanes, queue_size=10)
        self.pedestrian_pub = rospy.Publisher('/region/pedestrians', PedestrianStateArray, queue_size=10)

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

        # Ghost creation
        self._add_ghost_car = rospy.get_param("~ghost_car", False)
        self._ghost_pt = None
        self.restart_var = True
        self._publish_ghost = False
        self._prev_time_ghost = rospy.get_rostime()
        if (self._add_ghost_car):
            ghost_pt_param = rospy.get_param("~ghost_point")
            if ghost_pt_param:
                ghost_point = ghost_pt_param.split(',')
                if len(ghost_point) != 6:
                    raise ValueError("Invalid ghost_ point '{}'".format(ghost_pt_param))
                else:
                    self._ghost_pt = [float(ghost_point[0]), float(ghost_point[1]), float(ghost_point[2]), float(ghost_point[3]), float(ghost_point[4]), float(ghost_point[5])]

        if self._add_ghost_car and self._ghost_pt is None:
            raise NameError("No ghost points read")
            sys.exit(1)

    def __del__(self):
        self.vehicle_status_subscriber.unregister()
        self.ego_odom_subscriber.unregister()
        self.objects_subscriber.unregister()

    def vehicle_status_updated(self, vehicle_status):
        """
        Callback on vehicle status updates
        """
        self.ego_state.accel.accel = vehicle_status.acceleration

    def ego_odom_updated(self, odom):
        """
        Callback on vehicle status updates
        """
        self.ego_state.header = odom.header
        self.ego_state.pose.pose = odom.pose.pose
        self.ego_state.twist.twist = odom.twist.twist
        
        # Detection of the restart based on altitude of the car
        if self._add_ghost_car:
            if self.restart_var and odom.pose.pose.position.z < 0.2:
                print("Initial position started. Setting ghost")
                self.restart_var = False
                self._publish_ghost = True
                self._prev_time_ghost = rospy.get_rostime()
            elif not self.restart_var and odom.pose.pose.position.z >= 0.2:
                print("Restart detected")
                self.restart_var = True
                self._publish_ghost = False

            if self._publish_ghost:
                time_elapsed = (rospy.get_rostime() - self._prev_time_ghost).to_sec()
                if time_elapsed > 7.0:
                    self._publish_ghost = False
                    print("Stop publishing ghost")

        # Make copy of the dictionaries instead of using them across
        self.mutex_var.acquire()
        lanes_frenet_dict_cpy = self.lanes_frenet_dict
        self.mutex_var.release()

        if lanes_frenet_dict_cpy:

            ego_x = self.ego_state.pose.pose.position.x
            ego_y = self.ego_state.pose.pose.position.y
            quaternion = self.ego_state.pose.pose.orientation
            quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
            # yaw = -math.degrees(yaw)
            frenet = lanes_frenet_dict_cpy[ad_lane_val]
            ego_s, ego_d, conv = get_frenet(ego_x, ego_y, frenet[0], frenet[1])
            if not conv:
                return

            self.ego_state.s = ego_s
            self.ego_state.d = ego_d

            if self._add_localization_errors:
                # Compute error localization values and substitute them before sending
                s_new, d_new, v_new, yaw_new = self._error_localization.LocalizationUpdate(ego_s, ego_d, odom.twist.twist.linear.x, yaw)
                x_new, y_new, _ = get_xy(s_new, d_new, frenet[0], frenet[1])

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

    def objects_updated(self, msg):
        """
        Object callback        
        """
        object_array = msg.objects

        ########################################################################
        # Construct object array in map frame
        ########################################################################
        self.object_array_pub.publish(msg)

        ###############################################
        # no lane information
        ###############################################
        if not self.lanes_frenet_dict:
            return

        ###############################################
        # got lane information
        ###############################################
        # Make copy of the dictionaries instead of using them across
        self.mutex_var.acquire()
        lanes_frenet_dict_cpy = self.lanes_frenet_dict
        lanes_info_dict_cpy = self.lanes_info_dict
        self.mutex_var.release()

        perception_lanes_msg = PerceptionLanes()        
        pedestrian_msg = PedestrianStateArray()
        vehicles_dict_lane = {}
        for key in lanes_frenet_dict_cpy.keys():
            vehicles_dict_lane[key] = VehicleStateArray()        

        for i in range(len(object_array)):
            obj = object_array[i]

            if obj.pose.position.z < -1:
                continue

            obj_x = obj.pose.position.x
            obj_y = obj.pose.position.y
            obj_q = [obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w]
            
            _, _, obj_yaw = tf.transformations.euler_from_quaternion(obj_q)

            ego_x = self.ego_state.pose.pose.position.x
            ego_y = self.ego_state.pose.pose.position.y

            distance_to_ego = distance(ego_x, ego_y, obj_x, obj_y)

            if distance_to_ego > self.detect_range:
                continue

            # Pedestrians do not need lane classification
            if obj.classification == Object.CLASSIFICATION_PEDESTRIAN:
                ped = PedestrianState()
                ped.lifetime_id = obj.id
                ped.pose.pose = obj.pose
                ped.twist.twist = obj.twist
                pedestrian_msg.pedestrians.append(ped)
                continue

            # Figure out which lane the car is in
            valid_lanes = []
            for key, value in lanes_frenet_dict_cpy.items():                
                
                _ , val_d, val_conv = get_frenet(obj_x, obj_y, value[0], value[1])
                
                lane_info = lanes_info_dict_cpy[key]
               
                if val_conv and abs(val_d) <= lane_info.lane_width / 2.0:
                    valid_lanes.append(key)

            obj_lane = [-100]
            if len(valid_lanes) == 0:
                rospy.logwarn("Car should be at least in one lane")
                continue
            elif len(valid_lanes) == 1:
                obj_lane = [valid_lanes[0]]
            elif len(valid_lanes) == 2:
                if (valid_lanes[0] == target_lane_val and valid_lanes[1] == ad_lane_val) or (valid_lanes[0] == ad_lane_val and valid_lanes[1] == target_lane_val):
                    obj_lane = valid_lanes
                else:
                    #Decide the lane based on the headway
                    closest_ind1 = closest_point_ind(lanes_frenet_dict_cpy[valid_lanes[0]][0], obj_x, obj_y)
                    q1 = lanes_info_dict_cpy[valid_lanes[0]].path.poses[closest_ind1].pose.orientation
                    q1 = [q1.x, q1.y, q1.z, q1.w]
                    _, _, y1 = tf.transformations.euler_from_quaternion(q1)

                    closest_ind2 = closest_point_ind(lanes_frenet_dict_cpy[valid_lanes[1]][0], obj_x, obj_y)
                    q2 = lanes_info_dict_cpy[valid_lanes[1]].path.poses[closest_ind2].pose.orientation
                    q2 = [q2.x, q2.y, q2.z, q2.w]
                    _, _, y2 = tf.transformations.euler_from_quaternion(q2)

                    if abs(y1 - obj_yaw) <= abs(y2 - obj_yaw):
                        obj_lane = [valid_lanes[0]]
                    else:
                        obj_lane = [valid_lanes[1]]
            else:
                raise ValueError("Car cannot be in more than 2 lanes")
                continue

            veh = VehicleState()
            veh.header = obj.header
            veh.lifetime_id = obj.id
            veh.pose.pose = obj.pose
            veh.twist.twist = obj.twist
            veh.accel.accel = obj.accel
            veh.length = obj.shape.dimensions[0]
            veh.width = obj.shape.dimensions[1]

            for id_lane in obj_lane:

                frenet = lanes_frenet_dict_cpy[id_lane]
                veh.s, veh.d, veh_conv = get_frenet(obj_x, obj_y, frenet[0], frenet[1])

                send_car = False

                if self._add_perception_errors:
                    new_veh, send_car = self.check_and_add_noise(veh, frenet)

                    if send_car:
                        vehicles_dict_lane[id_lane].vehicles.append(new_veh)
                else:
                    vehicles_dict_lane[id_lane].vehicles.append(veh)

        if self._publish_ghost:
            veh_ghost = VehicleState()
            veh_ghost.lifetime_id = 123456789
            veh_ghost.pose.pose.position.x = self._ghost_pt[0]
            veh_ghost.pose.pose.position.y = -self._ghost_pt[1]
            veh_ghost.twist.twist.linear.x = 0.0
            veh_ghost.twist.twist.linear.y = 0.0
            veh_ghost.twist.twist.linear.z = 0.0
            veh_ghost.accel.accel.linear.x = 0.0
            veh_ghost.accel.accel.linear.y = 0.0
            veh_ghost.accel.accel.linear.z = 0.0
            q_new = tf.transformations.quaternion_from_euler(math.radians(self._ghost_pt[3]), math.radians(self._ghost_pt[4]), math.radians(self._ghost_pt[5]))
            veh_ghost.pose.pose.orientation.x = q_new[0]
            veh_ghost.pose.pose.orientation.y = q_new[1]
            veh_ghost.pose.pose.orientation.z = q_new[2]
            veh_ghost.pose.pose.orientation.w = q_new[3]

            veh_ghost.length = 5.0
            veh_ghost.width = 2.0
            
            frenet = lanes_frenet_dict_cpy[ad_lane_val]
            veh_ghost.s, veh_ghost.d, veh_conv = get_frenet(self._ghost_pt[0], -self._ghost_pt[1], frenet[0], frenet[1])

            vehicles_dict_lane[ad_lane_val].vehicles.append(veh_ghost)

        ###############################################
        # sort vehicle by s for each lane and publish
        ###############################################
        for key, value in vehicles_dict_lane.items():
            vehicles_msg = VehicleStateArray()
            if len(value.vehicles) > 1:
                vehicles_msg.vehicles = sorted(value.vehicles, key=attrgetter('s'), reverse=True)
            else:
                vehicles_msg.vehicles = value.vehicles
            perception_lanes_msg.ids.append(key)
            perception_lanes_msg.vehicles.append(vehicles_msg)

        perception_lanes_msg.header.frame_id = "map"
        pedestrian_msg.header.frame_id = "map"
        time_now = rospy.Time.now()
        perception_lanes_msg.header.stamp = time_now
        pedestrian_msg.header.stamp = time_now

        self.pedestrian_pub.publish(pedestrian_msg)
        self.perception_lanes_pub.publish(perception_lanes_msg)

    def check_and_add_noise(self, veh, frenet):

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
            x_new, y_new,_  = get_xy(s_new, d_new, frenet[0], frenet[1])

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

    def center_lanes_updated(self, msg):
        """
        Callback on gnss position updates        
        """

        self.mutex_var.acquire()

        num_lanes = len(msg.ids)
        self.lanes_frenet_dict = {}
        self.lanes_info_dict = {}

        for i in range(num_lanes):

            self.lanes_info_dict[msg.ids[i]] = LaneInfo(path=msg.center_lines[i], lane_width=msg.lanes_width[i], merging_wp=msg.crossing_ad_points[i])
            
            lane_line_list, lane_s_map = path_to_list(msg.center_lines[i])
            self.lanes_frenet_dict[msg.ids[i]] = [lane_line_list, lane_s_map]
        
        self.mutex_var.release()

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
        rate.sleep()


if __name__ == '__main__':

    main()
