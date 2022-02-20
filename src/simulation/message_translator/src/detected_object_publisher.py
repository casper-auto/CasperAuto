#!/usr/bin/env python

from utils.frenet_utils import *

from copy import deepcopy

import math
import rospy
import tf
import tf2_ros

from geometry_msgs.msg import Pose2D, Point
from derived_object_msgs.msg import Object, ObjectArray
from visualization_msgs.msg import Marker, MarkerArray
from traffic_msgs.msg import VehicleState, Prediction, PredictionArray

"""
class DetectedObjectPublisher
"""
class DetectedObjectPublisher(object):

    def __init__(self):

        '''
        Load parameters
        '''
        self.publish_object_cube = rospy.get_param("~publish_object_cube", False)
        self.publish_object_info = rospy.get_param("~publish_object_info", False)

        '''
        Variables
        '''
        self.curr_state = None
        self.curr_state_initialized = False

        self.whole_set = set()
        self.unshowed_set = set()

        '''
        Publishers
        '''
        self.detected_objects_pub = rospy.Publisher(
            "/region/detected_objects", ObjectArray, queue_size=10)
        self.detected_objects_marker_pub = rospy.Publisher(
            "/region/detected_objects_marker", MarkerArray, queue_size=10)
        self.predicted_paths_pub = rospy.Publisher(
            "/region/prediction/paths/display", MarkerArray, queue_size=10)

        '''
        Subscribers
        '''
        rospy.Subscriber(
            "/region/ego_state", VehicleState, self.ego_state_callback, queue_size=10)
        rospy.Subscriber(
            "/region/object_array", ObjectArray, self.objects_callback, queue_size=10)
        rospy.Subscriber(
            "/prediction/vehicles", PredictionArray, self.predicted_vehicles_callback, queue_size=10)
        rospy.Subscriber(
            "/prediction/pedestrians", PredictionArray, self.predicted_pedestrians_callback, queue_size=10)

        rospy.spin()

    """
    Callback functions
    """

    def ego_state_callback(self, msg):
        '''
        Callback on ego state updates
        '''
        rospy.loginfo("Received the ego state in detected object publisher.")

        self.curr_state = Pose2D()
        self.curr_state.x = msg.pose.pose.position.x
        self.curr_state.y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)
        self.curr_state.theta = yaw

        self.curr_state_initialized = True

    def objects_callback(self, msg):
        '''
        Callback on objects updates
        '''
        # rospy.loginfo("objects callback ..")

        self.unshowed_set = deepcopy(self.whole_set)

        object_array = ObjectArray()
        object_array.header.frame_id = "ego_vehicle"
        object_array.header.stamp = rospy.get_rostime()

        marker_array = MarkerArray()

        if not self.curr_state_initialized:
            return

        for obj in msg.objects:

            obj_id = obj.id

            self.whole_set.add(obj_id)
            self.unshowed_set.discard(obj_id)

            # global to local transformation
            obj_x = obj.pose.position.x
            obj_y = obj.pose.position.y
            # obj_z = obj.pose.position.z

            obj_quat = [obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w]
            _, _, obj_yaw = tf.transformations.euler_from_quaternion(obj_quat)

            obj_p_local = global_to_local(self.curr_state, self.curr_state.theta, Point2D(obj_x, obj_y))
            obj_yaw_local = obj_yaw - self.curr_state.theta

            obj_quat_local = tf.transformations.quaternion_from_euler(0, 0, obj_yaw_local)

            obj.pose.position.x = obj_p_local.x
            obj.pose.position.y = obj_p_local.y

            obj.pose.orientation.x = obj_quat_local[0]
            obj.pose.orientation.y = obj_quat_local[1]
            obj.pose.orientation.z = obj_quat_local[2]
            obj.pose.orientation.w = obj_quat_local[3]

            object_array.objects.append(obj)

            obj_vel = math.sqrt(obj.twist.linear.x ** 2 + obj.twist.linear.y ** 2)

            if self.publish_object_cube:
                # cube marker for the object shape
                marker = Marker()
                marker.header.frame_id = "ego_vehicle"
                marker.header.stamp = rospy.get_rostime()
                marker.id = obj_id
                marker.action = Marker.ADD
                marker.type = Marker.CUBE
                marker.pose.position.x = obj.pose.position.x
                marker.pose.position.y = obj.pose.position.y
                marker.pose.position.z = obj.pose.position.z + obj.shape.dimensions[2] / 2
                marker.pose.orientation.x = obj_quat_local[0]
                marker.pose.orientation.y = obj_quat_local[1]
                marker.pose.orientation.z = obj_quat_local[2]
                marker.pose.orientation.w = obj_quat_local[3]
                marker.scale.x = obj.shape.dimensions[0]
                marker.scale.y = obj.shape.dimensions[1]
                marker.scale.z = obj.shape.dimensions[2]

                if obj.classification == 6: # yellow for vehicles
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                elif obj.classification == 4: # orange for pedestrians
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 0.65
                    marker.color.b = 0.0
                else: # coral for else
                    marker.color.a = 0.7
                    marker.color.r = 0.7
                    marker.color.g = 0.3
                    marker.color.b = 0.0

                marker_array.markers.append(marker)

            if self.publish_object_info:
                # arrow marker for the object heading
                marker = Marker()
                marker.header.frame_id = "ego_vehicle"
                marker.header.stamp = rospy.get_rostime()
                marker.id = obj_id + 1000
                marker.action = Marker.ADD
                marker.type = Marker.ARROW
                marker.pose.position.x = obj.pose.position.x
                marker.pose.position.y = obj.pose.position.y
                marker.pose.position.z = obj.pose.position.z + obj.shape.dimensions[2] / 2 + 2.5
                marker.pose.orientation.x = obj_quat_local[0]
                marker.pose.orientation.y = obj_quat_local[1]
                marker.pose.orientation.z = obj_quat_local[2]
                marker.pose.orientation.w = obj_quat_local[3]
                marker.scale.x = 3.0
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 0.5
                marker.color.g = 0.6
                marker.color.b = 1.0

                marker_array.markers.append(marker)

                # text marker for the object speed
                marker = Marker()
                marker.header.frame_id = "ego_vehicle"
                marker.header.stamp = rospy.get_rostime()
                marker.id = obj_id + 2000
                marker.action = Marker.ADD
                marker.type = Marker.TEXT_VIEW_FACING
                marker.pose.position.x = obj.pose.position.x - 2.0
                marker.pose.position.y = obj.pose.position.y
                marker.pose.position.z = obj.pose.position.z + obj.shape.dimensions[2] / 2 + 2.0
                marker.pose.orientation.x = obj_quat_local[0]
                marker.pose.orientation.y = obj_quat_local[1]
                marker.pose.orientation.z = obj_quat_local[2]
                marker.pose.orientation.w = obj_quat_local[3]
                marker.text = str(round(obj_vel, 2)) + "m/s"
                marker.scale.z = 2.0
                marker.color.a = 1.0
                marker.color.r = 0.5
                marker.color.g = 0.6
                marker.color.b = 1.0

                marker_array.markers.append(marker)

                # text marker for lane id and object id display
                marker = Marker()
                marker.header.frame_id = "ego_vehicle"
                marker.header.stamp = rospy.get_rostime()
                marker.id = obj_id + 3000
                marker.action = Marker.ADD
                marker.type = Marker.TEXT_VIEW_FACING
                marker.pose.position.x = obj.pose.position.x
                marker.pose.position.y = obj.pose.position.y
                marker.pose.position.z = obj.pose.position.z + obj.shape.dimensions[2] / 2 + 2.0
                marker.pose.orientation.x = obj_quat_local[0]
                marker.pose.orientation.y = obj_quat_local[1]
                marker.pose.orientation.z = obj_quat_local[2]
                marker.pose.orientation.w = obj_quat_local[3]
                marker.text = str(obj_id)
                marker.scale.z = 2.0
                marker.color.a = 1.0
                marker.color.r = 0.5
                marker.color.g = 0.6
                marker.color.b = 1.0

                marker_array.markers.append(marker)

        for id in self.unshowed_set:

            if self.publish_object_cube:
                # remove cube marker
                marker = Marker()
                marker.header.frame_id = "ego_vehicle"
                marker.header.stamp = rospy.get_rostime()
                marker.id = id
                marker.action = Marker.DELETE

                marker_array.markers.append(marker)

            if self.publish_object_info:
                # remove arrow marker
                marker = Marker()
                marker.header.frame_id = "ego_vehicle"
                marker.header.stamp = rospy.get_rostime()
                marker.id = id + 1000
                marker.action = Marker.DELETE

                # remove id text marker
                marker_array.markers.append(marker)
                marker = Marker()
                marker.header.frame_id = "ego_vehicle"
                marker.header.stamp = rospy.get_rostime()
                marker.id = id + 2000
                marker.action = Marker.DELETE

                # remove id text marker
                marker_array.markers.append(marker)
                marker = Marker()
                marker.header.frame_id = "ego_vehicle"
                marker.header.stamp = rospy.get_rostime()
                marker.id = id + 3000
                marker.action = Marker.DELETE

                marker_array.markers.append(marker)

        self.detected_objects_pub.publish(object_array)
        self.detected_objects_marker_pub.publish(marker_array)

    def predicted_vehicles_callback(self, msg):

        predicted_lines = MarkerArray()

        for pred in msg.predictions:

            # for each object
            predicted_line = Marker()
            predicted_line.lifetime = rospy.Duration(0.2)
            predicted_line.header.frame_id = "ego_vehicle"
            predicted_line.header.stamp = rospy.get_rostime()
            predicted_line.ns = "predicted_line"
            predicted_line.id = pred.agent_id
            predicted_line.action = Marker.ADD
            predicted_line.type = Marker.LINE_STRIP

            predicted_line.pose.orientation.w = 1.0
            predicted_line.scale.x = 2
            predicted_line.color.r = 1.0
            predicted_line.color.g = 1.0
            predicted_line.color.a = 0.5

            for wp in pred.trajectories[0].trajectory_estimated.waypoints:

                if self.curr_state_initialized:
                    # convert to local
                    p = Point2D(wp.pose.pose.position.x, wp.pose.pose.position.y)
                    p_local = global_to_local(self.curr_state, self.curr_state.theta, p)

                    pt = Point()
                    pt.x = p_local.x
                    pt.y = p_local.y
                    predicted_line.points.append(pt)

            predicted_lines.markers.append(predicted_line)

        self.predicted_paths_pub.publish(predicted_lines)

    def predicted_pedestrians_callback(self, msg):

        predicted_lines = MarkerArray()

        for pred in msg.predictions:

            # for each object
            predicted_line = Marker()
            predicted_line.lifetime = rospy.Duration(0.2)
            predicted_line.header.frame_id = "ego_vehicle"
            predicted_line.header.stamp = rospy.get_rostime()
            predicted_line.ns = "predicted_line"
            predicted_line.id = pred.agent_id
            predicted_line.action = Marker.ADD
            predicted_line.type = Marker.LINE_STRIP

            predicted_line.pose.orientation.w = 1.0
            predicted_line.scale.x = 1.0
            predicted_line.color.r = 1.0
            predicted_line.color.g = 0.65
            predicted_line.color.a = 0.5

            for wp in pred.trajectories[0].trajectory_estimated.waypoints:

                if self.curr_state_initialized:
                    # convert to local
                    p = Point2D(wp.pose.pose.position.x, wp.pose.pose.position.y)
                    p_local = global_to_local(self.curr_state, self.curr_state.theta, p)

                    pt = Point()
                    pt.x = p_local.x
                    pt.y = p_local.y
                    predicted_line.points.append(pt)

            predicted_lines.markers.append(predicted_line)

        self.predicted_paths_pub.publish(predicted_lines)


"""
main function
"""
def main():
    rospy.init_node('detected_object_visualizer_node', anonymous=True)
    node = DetectedObjectPublisher()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        node.publish_customized_msg()
        rate.sleep()

if __name__ == '__main__':
    main()
