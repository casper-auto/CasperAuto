#!/usr/bin/env python

from __future__ import print_function

# ros
import rospy
import tf
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose2D, Point, Pose
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
# traffic_msgs
from traffic_msgs.msg import VehicleState, VehicleStateArray
# honda_protocol_msgs
from honda_protocol_msgs.msg import LocalizationTextInfo
# utils
from utils.frenet_utils import *

"""
class StopSign
"""
class StopSign(object):

    def __init__(self, pt, s, theta, timer_start, timer_activated):

        self.pt = pt
        self.s = s
        self.theta = theta
        self.timer_start = timer_start
        self.timer_activated = timer_activated

"""
class StopSignProcessor
"""
class StopSignProcessor(object):

    def __init__(self):

        '''
        Load parameters
        '''
        self.stop_line_offset = rospy.get_param("~stop_line_offset", 2.5)
        self.stop_dist_thresh = rospy.get_param("~stop_dist_thresh", 5.0)
        self.stop_speed_thresh = rospy.get_param("~stop_speed_thresh", 0.5)
        self.stop_timer_duration = rospy.get_param("~stop_timer_duration", 2.5)
        self.publish_ghost = rospy.get_param("~publish_ghost", False)

        '''
        Variables
        '''
        self.prev_state = None
        self.curr_state = None
        self.ego_state = VehicleState()
        self.ego_state_initialized = False

        self.ref_path_list = None
        self.ref_s_map = None
        self.ref_path_initialized = False

        self.stop_signs = {}
        self.ghost_vehicles = VehicleStateArray()

        '''
        Publishers
        '''
        self.stop_timer_marker_pub = rospy.Publisher('/region/stop_timer_display', Marker, queue_size=10)
        self.stop_barrier_marker_pub = rospy.Publisher('/region/stop_barrier_display', Marker, queue_size=10)
        self.ghost_vehicles_pub = rospy.Publisher('/region/ghost_vehicles', VehicleStateArray, queue_size=10)
        self.ghost_marker_pub = rospy.Publisher('/region/ghost_marker', Marker, queue_size=10)

        '''
        Subscribers
        '''
        self.ego_state_subscriber = rospy.Subscriber(
            "/region/ego_state", VehicleState, self.ego_state_callback, queue_size=10)
        self.global_route_subscriber = rospy.Subscriber(
            "/region/global_route", Path, self.global_route_callback, queue_size=10)
        self.stop_lines_marker_subscriber = rospy.Subscriber(
            '/region/odr_map/stoplines_marker', MarkerArray, self.stoplines_callback, queue_size=10)

    """
    Callback functions
    """

    def ego_state_callback(self, msg):
        # rospy.loginfo("Received the ego state in stop sign processor.")

        self.curr_state = Pose2D()
        self.curr_state.x = msg.pose.pose.position.x
        self.curr_state.y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)
        self.curr_state.theta = yaw

        self.ego_state = msg

        # Check if need to reset
        if self.prev_state:
            if distance(self.prev_state.x, self.prev_state.y, self.curr_state.x, self.curr_state.y) > 5.0:
                self.reset()

        self.prev_state = self.curr_state

        self.ego_state_initialized = True

    def global_route_callback(self, msg):
        # rospy.loginfo("Received the global route in stop sign processor.")

        self.global_route = msg
        self.ref_path_list, self.ref_s_map = path_to_list(self.global_route)

        self.ref_path_initialized = True

    def stoplines_callback(self, msg):
        # rospy.loginfo("Received the stop lines in stop sign processor.")

        if self.ego_state_initialized and self.ref_path_initialized:

            for marker in msg.markers:

                stop_id = marker.id

                if stop_id not in self.stop_signs.keys():

                    points = marker.points
                    if len(points) < 2:
                        return

                    pl = points[0]
                    pr = points[-1]
                    stop_pt = Point2D((pl.x + pr.x)/2.0, (pl.y + pr.y)/2.0)

                    stop_s, _, stop_theta, conv = get_frenet_with_theta(stop_pt.x, stop_pt.y, self.ref_path_list, self.ref_s_map)

                    self.stop_signs[stop_id] = StopSign(pt=stop_pt,
                                                        s=stop_s,
                                                        theta=stop_theta,
                                                        timer_start=None,
                                                        timer_activated=False)

    """
    Helper functions
    """

    def reset(self):
        rospy.loginfo("Resetting stop sign.")

        self.prev_state = None
        self.ego_state_initialized = False

        # Delete all markers
        for stop_id in self.stop_signs.keys():
            self.publish_ghost_marker(stop_id, None)

        self.stop_signs = {}

    def update(self):
        '''
        Process the next stop sign if exist
        '''
        # rospy.loginfo("Stop sign update.")

        if self.ego_state_initialized and self.ref_path_initialized:

            self.ghost_vehicles = VehicleStateArray()
            self.ghost_vehicles.header.stamp = rospy.get_rostime()
            self.ghost_vehicles.header.frame_id = "map"

            for stop_id, stop_sign in self.stop_signs.items():

                rospy.loginfo("stop_id = %d", stop_id)
                rospy.loginfo(stop_sign.s)
                rospy.loginfo(self.ego_state.s)

                ego_bumper_s = self.ego_state.s + self.ego_state.length/2.0

                # Idea: speed <= 3.0, stop_dist = 5 m (bumper to bumper)
                #       speed > 3.0, stop dist = 5 m + (speed^2 - 3^2) / (2 * 1.0)
                #       (assuming deceleration is 1 m/s^2)
                stop_dist = self.stop_dist_thresh \
                    + max((self.ego_state.twist.twist.linear.x ** 2- 9) / 2.0, 0)

                # check if still far to decelerate
                if stop_sign.s - ego_bumper_s > stop_dist:
                    continue
                # check if already passed
                if stop_sign.s + 1.0 < ego_bumper_s:
                    self.publish_timer_marker(None, None, True)
                    self.publish_barrier_marker(stop_id, None, True)
                    self.publish_ghost_marker(stop_id, None)
                    continue
                # check if activated
                if stop_sign.timer_activated:
                    timer_elapsed = (rospy.get_rostime() - stop_sign.timer_start).to_sec()
                    if timer_elapsed < self.stop_timer_duration:
                        stop_pose = self.create_pose(stop_sign.pt, stop_sign.theta)
                        stop_pose_local = self.get_local_pose(stop_pose)
                        self.publish_timer_marker(stop_sign.pt, self.stop_timer_duration - timer_elapsed)
                        self.publish_barrier_marker(stop_id, stop_pose_local)
                        # adjust ghost vehicle position by shiftting in backward direction
                        pt, theta = self.shift_backward(stop_sign.pt, stop_sign.theta, self.stop_line_offset)
                        stop_pose = self.create_pose(pt, theta)
                        stop_pose_local = self.get_local_pose(stop_pose)
                        self.append_ghost_vehicle(stop_id, stop_pose, stop_sign.s)
                        self.publish_ghost_marker(stop_id, stop_pose_local)
                    else:
                        self.publish_timer_marker(None, None, True)
                        self.publish_barrier_marker(stop_id, None, True)
                        self.publish_ghost_marker(stop_id, None)
                else:
                    stop_pose = self.create_pose(stop_sign.pt, stop_sign.theta)
                    stop_pose_local = self.get_local_pose(stop_pose)
                    self.publish_barrier_marker(stop_id, stop_pose_local)
                    # adjust ghost vehicle position by shiftting in backward direction
                    pt, theta = self.shift_backward(stop_sign.pt, stop_sign.theta, self.stop_line_offset)
                    stop_pose = self.create_pose(pt, theta)
                    stop_pose_local = self.get_local_pose(stop_pose)
                    self.append_ghost_vehicle(stop_id, stop_pose, stop_sign.s)
                    self.publish_ghost_marker(stop_id, stop_pose_local)

                    # check if need to activate now (position and speed)
                    distance = stop_sign.s - ego_bumper_s
                    vx = self.ego_state.twist.twist.linear.x
                    vy = self.ego_state.twist.twist.linear.x
                    speed = math.sqrt(vx*vx + vy*vy)
                    if distance < self.stop_dist_thresh and speed < self.stop_speed_thresh:
                        stop_sign.timer_activated = True
                        stop_sign.timer_start = rospy.get_rostime()

            if self.publish_ghost:
                self.ghost_vehicles_pub.publish(self.ghost_vehicles)

    def publish_timer_marker(self, pos, remained_time, expired=False):
        if not expired:
            pos_local = global_to_local(self.curr_state, self.curr_state.theta, pos)
            marker = Marker()
            marker.header.frame_id = "ego_vehicle"
            marker.header.stamp = rospy.get_rostime()
            marker.id = 0
            marker.type = Marker.TEXT_VIEW_FACING
            marker.pose.position.x = pos_local.x
            marker.pose.position.y = pos_local.y + 4.0
            marker.text = str(int(remained_time))
            marker.scale.z = 5
            marker.color.a = 0.75
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.5
        else:
            marker = Marker()
            marker.header.frame_id = "ego_vehicle"
            marker.header.stamp = rospy.get_rostime()
            marker.id = 0
            marker.action = Marker.DELETE

        self.stop_timer_marker_pub.publish(marker)

    def publish_barrier_marker(self, id, pose, expired=False):
        # display in "ego_vehicle" frame
        if not expired:
            # marker
            marker = Marker()
            marker.header.stamp = rospy.get_rostime()
            marker.header.frame_id = "ego_vehicle"
            marker.id = id
            marker.action = Marker.ADD
            marker.type = Marker.CUBE
            marker.pose = pose
            marker.scale.x = 0.2
            marker.scale.y = 4.0
            marker.scale.z = 2.5
            marker.color.a = 0.75
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.5
            self.stop_barrier_marker_pub.publish(marker)
        else:
            # remove ghost marker
            marker = Marker()
            marker.header.stamp = rospy.get_rostime()
            marker.header.frame_id = "ego_vehicle"
            marker.id = id
            marker.action = Marker.DELETE
            self.stop_barrier_marker_pub.publish(marker)

    def append_ghost_vehicle(self, id, pose, s):
        '''
        Append ghost vehicle and marker
        '''
        # vehicle state
        veh_ghost = VehicleState()
        veh_ghost.header.stamp = rospy.get_rostime()
        veh_ghost.header.frame_id = "map"
        veh_ghost.lifetime_id = 123456789
        veh_ghost.local_id = id
        veh_ghost.pose.pose = pose
        veh_ghost.twist.twist.linear.x = 0.0
        veh_ghost.twist.twist.linear.y = 0.0
        veh_ghost.twist.twist.linear.z = 0.0
        veh_ghost.accel.accel.linear.x = 0.0
        veh_ghost.accel.accel.linear.y = 0.0
        veh_ghost.accel.accel.linear.z = 0.0
        veh_ghost.length = 5.0
        veh_ghost.width = 2.0
        veh_ghost.s = s
        veh_ghost.d = 0.0

        # Append or replace
        if len(self.ghost_vehicles.vehicles) == 0:
            self.ghost_vehicles.vehicles.append(veh_ghost)
        elif s < self.ghost_vehicles.vehicles[0].s:
            self.ghost_vehicles.vehicles[0] = veh_ghost

    def publish_ghost_marker(self, id, pose):
        # display in "ego_vehicle" frame
        if pose:
            # marker
            marker = Marker()
            marker.header.stamp = rospy.get_rostime()
            marker.header.frame_id = "ego_vehicle"
            marker.id = id
            marker.action = Marker.ADD
            marker.type = Marker.CUBE
            marker.pose = pose
            marker.scale.x = 5.0
            marker.scale.y = 2.0
            marker.scale.z = 2.5
            marker.color.a = 1.0
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
            self.ghost_marker_pub.publish(marker)
        else:
            # remove ghost marker
            marker = Marker()
            marker.header.stamp = rospy.get_rostime()
            marker.header.frame_id = "ego_vehicle"
            marker.id = id
            marker.action = Marker.DELETE
            self.ghost_marker_pub.publish(marker)

    def create_pose(self, pt, theta):
        # Pose
        pose = Pose()
        pose.position.x = pt.x
        pose.position.y = pt.y
        pose.position.z = 1.0
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    def get_local_pose(self, pose_global):
        '''
        Transform to local pose (rviz has difficulty to show in "map")
        '''

        # point
        p = pose_global.position
        p = global_to_local(self.curr_state, self.curr_state.theta, p)
        pose_local = Pose()
        pose_local.position = Point(p.x, p.y, 1.0)

        # yaw
        quaternion = (
            pose_global.orientation.x,
            pose_global.orientation.y,
            pose_global.orientation.z,
            pose_global.orientation.w
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw - self.curr_state.theta)
        pose_local.orientation.x = quaternion[0]
        pose_local.orientation.y = quaternion[1]
        pose_local.orientation.z = quaternion[2]
        pose_local.orientation.w = quaternion[3]

        return pose_local

    def shift_backward(self, pt, theta, offset):
        raw_s, _, raw_theta, conv = get_frenet_with_theta(pt.x, pt.y, self.ref_path_list, self.ref_s_map)
        shifted_s = raw_s + offset
        px, py, conv = get_xy(shifted_s, 0.0, self.ref_path_list, self.ref_s_map)
        shifted_pt = Point2D(px, py)
        shifted_theta = raw_theta
        return shifted_pt, shifted_theta

"""
main function
"""
def main():
    rospy.init_node('stop_sign_processor_node', anonymous=True)
    processor = StopSignProcessor()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        processor.update()
        rate.sleep()

if __name__ == '__main__':
    main()
