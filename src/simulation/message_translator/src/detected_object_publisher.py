#!/usr/bin/env python
from copy import deepcopy
import math

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Pose2D, Point, PoseStamped
from derived_object_msgs.msg import Object, ObjectArray
from visualization_msgs.msg import Marker, MarkerArray

from utils.geometry_utils import *


'''
class DetectedObjectPublisher
'''
class DetectedObjectPublisher(object):

    def __init__(self):

        '''
        Load parameters
        '''
        self.publish_object_cube = rospy.get_param('~publish_object_cube', False)
        self.publish_object_info = rospy.get_param('~publish_object_info', False)

        '''
        Variables
        '''
        self.curr_state = None
        self.curr_state_initialized = False

        self.whole_set = set()
        self.unshowed_set = set()

        '''
        Subscribers
        '''
        rospy.Subscriber(
            '/casper_auto/current_pose', PoseStamped, self.current_pose_callback, queue_size=10)
        rospy.Subscriber(
            '/casper_auto/detected_objects', ObjectArray, self.detected_objects_callback, queue_size=10)

        '''
        Publishers
        '''
        self.detected_objects_marker_pub = rospy.Publisher(
            '/casper_auto/detected_objects_marker', MarkerArray, queue_size=10)

        rospy.spin()

    '''
    Callback functions
    '''

    def current_pose_callback(self, msg):
        rospy.loginfo('Received current_pose in detected object publisher.')

        self.curr_state = Pose2D()
        self.curr_state.x = msg.pose.position.x
        self.curr_state.y = msg.pose.position.y

        quat = msg.pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)
        self.curr_state.theta = yaw

        self.curr_state_initialized = True

    def detected_objects_callback(self, msg):
        rospy.loginfo('Received detected_objects in detected object publisher.')

        marker_array = MarkerArray()

        for obj in msg.objects:

            self.whole_set.add(obj.id)
            self.unshowed_set.discard(obj.id)

            if self.publish_object_cube:
                # cube marker for the object shape
                marker = Marker()
                marker.header.frame_id = obj.header.frame_id
                marker.header.stamp = rospy.get_rostime()
                marker.id = obj.id
                marker.action = Marker.ADD
                marker.type = Marker.CUBE
                marker.pose.position.x = obj.pose.position.x
                marker.pose.position.y = obj.pose.position.y
                marker.pose.position.z = obj.pose.position.z
                marker.pose.orientation.x = obj.pose.orientation.x
                marker.pose.orientation.y = obj.pose.orientation.y
                marker.pose.orientation.z = obj.pose.orientation.z
                marker.pose.orientation.w = obj.pose.orientation.w
                marker.scale.x = obj.shape.dimensions[0]
                marker.scale.y = obj.shape.dimensions[1]
                marker.scale.z = obj.shape.dimensions[2]

                if obj.classification == 6: # yellow for vehicles
                    marker.color.a = 0.5
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                elif obj.classification == 4: # orange for pedestrians
                    marker.color.a = 0.5
                    marker.color.r = 1.0
                    marker.color.g = 0.65
                    marker.color.b = 0.0
                else: # coral for else
                    marker.color.a = 0.5
                    marker.color.r = 0.7
                    marker.color.g = 0.3
                    marker.color.b = 0.0

                marker_array.markers.append(marker)

            if self.publish_object_info:
                # arrow marker for the object heading
                marker = Marker()
                marker.header.frame_id = obj.header.frame_id
                marker.header.stamp = rospy.get_rostime()
                marker.id = obj.id + 1000
                marker.action = Marker.ADD
                marker.type = Marker.ARROW
                marker.pose.position.x = obj.pose.position.x
                marker.pose.position.y = obj.pose.position.y
                marker.pose.position.z = obj.pose.position.z + 2.5
                marker.pose.orientation.x = obj.pose.orientation.x
                marker.pose.orientation.y = obj.pose.orientation.y
                marker.pose.orientation.z = obj.pose.orientation.z
                marker.pose.orientation.w = obj.pose.orientation.w
                marker.scale.x = 3.0
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                marker_array.markers.append(marker)

                # text marker for the object speed
                obj_vel = math.sqrt(obj.twist.linear.x ** 2 + obj.twist.linear.y ** 2 + obj.twist.linear.z ** 2)
                marker = Marker()
                marker.header.frame_id = obj.header.frame_id
                marker.header.stamp = rospy.get_rostime()
                marker.id = obj.id + 2000
                marker.action = Marker.ADD
                marker.type = Marker.TEXT_VIEW_FACING
                marker.pose.position.x = obj.pose.position.x - 2.0
                marker.pose.position.y = obj.pose.position.y
                marker.pose.position.z = obj.pose.position.z + 2.0
                marker.pose.orientation.x = obj.pose.orientation.x
                marker.pose.orientation.y = obj.pose.orientation.y
                marker.pose.orientation.z = obj.pose.orientation.z
                marker.pose.orientation.w = obj.pose.orientation.w
                marker.text = str(round(obj_vel, 2)) + 'm/s'
                marker.scale.z = 2.0
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.4
                marker.color.b = 0.4

                marker_array.markers.append(marker)

                # text marker for lane id and object id display
                marker = Marker()
                marker.header.frame_id = obj.header.frame_id
                marker.header.stamp = rospy.get_rostime()
                marker.id = obj.id + 3000
                marker.action = Marker.ADD
                marker.type = Marker.TEXT_VIEW_FACING
                marker.pose.position.x = obj.pose.position.x
                marker.pose.position.y = obj.pose.position.y
                marker.pose.position.z = obj.pose.position.z + 2.0
                marker.pose.orientation.x = obj.pose.orientation.x
                marker.pose.orientation.y = obj.pose.orientation.y
                marker.pose.orientation.z = obj.pose.orientation.z
                marker.pose.orientation.w = obj.pose.orientation.w
                marker.text = str(obj.id)
                marker.scale.z = 2.0
                marker.color.a = 1.0
                marker.color.r = 0.5
                marker.color.g = 1.0
                marker.color.b = 0.6

                marker_array.markers.append(marker)

        for id in self.unshowed_set:

            if self.publish_object_cube:
                # remove cube marker
                marker = Marker()
                marker.header.frame_id = obj.header.frame_id
                marker.header.stamp = rospy.get_rostime()
                marker.id = id
                marker.action = Marker.DELETE

                marker_array.markers.append(marker)

            if self.publish_object_info:
                # remove arrow marker
                marker = Marker()
                marker.header.frame_id = obj.header.frame_id
                marker.header.stamp = rospy.get_rostime()
                marker.id = id + 1000
                marker.action = Marker.DELETE

                # remove id text marker
                marker_array.markers.append(marker)
                marker = Marker()
                marker.header.frame_id = obj.header.frame_id
                marker.header.stamp = rospy.get_rostime()
                marker.id = id + 2000
                marker.action = Marker.DELETE

                # remove id text marker
                marker_array.markers.append(marker)
                marker = Marker()
                marker.header.frame_id = obj.header.frame_id
                marker.header.stamp = rospy.get_rostime()
                marker.id = id + 3000
                marker.action = Marker.DELETE

                marker_array.markers.append(marker)

        self.detected_objects_marker_pub.publish(marker_array)


'''
main function
'''
def main():
    rospy.init_node('detected_object_visualizer_node', anonymous=True)
    node = DetectedObjectPublisher()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
