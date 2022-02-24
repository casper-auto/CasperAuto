#!/usr/bin/env python

from __future__ import print_function

# ros
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Pose2D, PoseStamped
from derived_object_msgs.msg import Object, ObjectArray

# utils
from utils.geometry_utils import *


class MessageTranslator(object):

    '''
    The class will maintain a system of the local traffic which holds the
    information for the ego vehicle to make the decision in the lane change
    process.
    '''

    def __init__(self, debug=False):

        '''
        Load parameters
        '''
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')
        self.detect_range = rospy.get_param('~detect_range', 60.0)

        '''
        Variables
        '''
        self.debug = debug
        self.vehicle_frame_id = self.role_name
        self.curr_state = Pose2D()
        self.curr_z = 0

        self.detected_objects = None

        '''
        Subscribers
        '''
        self.carla_odometry_subscriber = rospy.Subscriber(
            '/carla/ego_vehicle/odometry', Odometry, self.carla_odometry_callback, queue_size=10)
        self.carla_speedometer_subscriber = rospy.Subscriber(
            '/carla/ego_vehicle/speedometer', Float32, self.carla_speedometer_callback, queue_size=10)
        self.objects_subscriber = rospy.Subscriber(
            '/carla/ego_vehicle/objects', ObjectArray, self.carla_objects_callback, queue_size=10)

        '''
        Publishers
        '''
        self.current_pose_pub = rospy.Publisher('/casper_auto/current_pose', PoseStamped, queue_size=10)
        self.current_speed_pub = rospy.Publisher('/casper_auto/current_speed', Float32, queue_size=10)
        self.detected_object_array_pub = rospy.Publisher('/casper_auto/detected_objects', ObjectArray, queue_size=10)

    '''
    Callback functions
    '''

    def carla_odometry_callback(self, msg):
        rospy.loginfo('Received carla odometry in message translator.')

        '''
        Update current vehicle state in 2d
        '''
        self.curr_state.x = msg.pose.pose.position.x
        self.curr_state.y = msg.pose.pose.position.y
        self.curr_z = msg.pose.pose.position.z

        # transform quaternion to rpy
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        self.curr_state.theta = yaw

        '''
        Publish current pose in map frame
        '''
        self.current_pose = PoseStamped()
        self.current_pose.header.stamp = rospy.get_rostime()
        self.current_pose.header.frame_id = 'map'
        self.current_pose.pose = msg.pose.pose
        self.current_pose_pub.publish(self.current_pose)

    def carla_speedometer_callback(self, msg):
        rospy.loginfo('Received carla speedometer speed in message translator.')
        self.current_speed_pub.publish(msg)

    def carla_objects_callback(self, msg):
        rospy.loginfo('Received carla objects in message translator.')

        '''
        Construct object array in local / vehicle frame
        '''
        object_array = ObjectArray()
        object_array.header.stamp = rospy.get_rostime()
        object_array.header.frame_id = self.vehicle_frame_id

        for obj in msg.objects:

            obj.header.stamp = rospy.get_rostime()
            obj.header.frame_id = self.vehicle_frame_id

            # transform quaternion to rpy
            quaternion = (
                obj.pose.orientation.x,
                obj.pose.orientation.y,
                obj.pose.orientation.z,
                obj.pose.orientation.w
            )
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            obj_yaw = yaw

            # transform from global to local
            p = Point2D(obj.pose.position.x, obj.pose.position.y)
            p = global_to_local(self.curr_state, self.curr_state.theta, p)
            theta = obj_yaw - self.curr_state.theta

            obj.pose.position.x = p.x
            obj.pose.position.y = p.y
            obj.pose.position.z = obj.pose.position.z - 1.5

            quaternion = quaternion_from_euler(roll, pitch, theta)
            obj.pose.orientation.x = quaternion[0]
            obj.pose.orientation.y = quaternion[1]
            obj.pose.orientation.z = quaternion[2]
            obj.pose.orientation.w = quaternion[3]

            object_array.objects.append(obj)

        self.detected_object_array_pub.publish(object_array)


'''
main function
'''

def main():
    rospy.init_node('message_translator', anonymous=True)
    node = MessageTranslator(debug=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
