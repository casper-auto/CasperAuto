#!/usr/bin/env python

from __future__ import print_function

import numpy as np

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
        self.world_frame = 'map'
        self.vehicle_frame = self.role_name
        self.curr_state = Pose2D()
        self.curr_z = 0

        self.detected_objects = None

        '''
        TF Listener
        '''
        self.listener = tf.TransformListener()

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
        Publish current pose in world frame
        '''
        self.current_pose = PoseStamped()
        self.current_pose.header.stamp = rospy.get_rostime()
        self.current_pose.header.frame_id = self.world_frame
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
        object_array.header.frame_id = self.vehicle_frame

        for obj in msg.objects:

            obj.header.stamp = rospy.get_rostime()
            obj.header.frame_id = self.vehicle_frame

            pose_mat = self.from_pose_msg_to_matrix(obj.pose)
            transformed = self.world_to_local(pose_mat)

            trans = tf.transformations.translation_from_matrix(transformed)
            quat = tf.transformations.quaternion_from_matrix(transformed)

            obj.pose.position.x = trans[0]
            obj.pose.position.y = trans[1]
            obj.pose.position.z = trans[2] # + obj.shape.dimensions[2]/2.0

            if obj.classification == obj.CLASSIFICATION_CAR:
                obj.pose.position.z += obj.shape.dimensions[2]/2.0

            obj.pose.orientation.x = quat[0]
            obj.pose.orientation.y = quat[1]
            obj.pose.orientation.z = quat[2]
            obj.pose.orientation.w = quat[3]

            object_array.objects.append(obj)

        self.detected_object_array_pub.publish(object_array)

    '''
    Helper functions
    '''
    def from_pose_msg_to_matrix(self, pose):
        trans = tf.transformations.translation_matrix((pose.position.x,
                                                       pose.position.y,
                                                       pose.position.z))

        rot = tf.transformations.quaternion_matrix((pose.orientation.x,
                                                    pose.orientation.y,
                                                    pose.orientation.z,
                                                    pose.orientation.w))

        # rospy.loginfo("trans: " + str(trans))
        # rospy.loginfo("rot: " + str(rot))

        combined = np.matmul(trans, rot)
        return combined

    def world_to_local(self, pose):
        """
        Transform a pose from world frame to local frame
        @param pose The 4x4 transformation matrix containing the pose to transform
        @return The 4x4 transformation matrix describing the pose in local frame
        """
        # Get pose w.r.t world frame
        self.listener.waitForTransform(self.vehicle_frame, self.world_frame,
                                       rospy.Time(), rospy.Duration(10))
        t, r = self.listener.lookupTransform(self.vehicle_frame, self.world_frame,
                                             rospy.Time(0))

        # Get relative transform between frames
        offset = np.matrix(tf.transformations.quaternion_matrix(r))
        offset[0,3] = t[0]
        offset[1,3] = t[1]
        offset[2,3] = t[2]

        # Compose with pose to get pose in local frame
        result = np.array(np.dot(offset, pose))

        return result

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
