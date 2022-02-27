#!/usr/bin/env python

"""
Traffic light visualize and query.
"""
import math
import threading
import random

import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32, ColorRGBA
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from visualization_msgs.msg import Marker, MarkerArray

from carla_msgs.msg import CarlaTrafficLightInfoList, CarlaTrafficLightStatusList
from carla_tf_light.srv import GetNextTrafficLight, GetNextTrafficLightRequest

import carla
import carla_common.transforms as trans
import ros_compatibility as roscomp


class TfLightTest(object):

    """
    Publish traffc light in rviz.
    """

    RED = ColorRGBA(1.0, 0.0, 0.0, 1.0)
    YELLOW = ColorRGBA(1.0, 1.0, 0.0, 1.0)
    GREEN = ColorRGBA(0.0, 1.0, 0.0, 1.0)
    WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)

    def __init__(self):
        '''
        Vars
        '''
        self.current_pose = Pose()
        self.next_tf_light = {'state': 4, 'stop_line_location': Point()}

        '''
        ROS Publishers
        '''
        self.next_tf_light_marker_publisher = rospy.Publisher(
            '/casper_auto/traffic_light_test', Marker, queue_size=10)


        '''
        ROS Service
        '''
        rospy.wait_for_service("/casper_auto/next_tf_light")
        self.get_tf_light_service = rospy.ServiceProxy("/casper_auto/next_tf_light", GetNextTrafficLight)

        print('-------------')

        '''
        ROS Subscribers
        '''
        self.current_pose_subscriber = rospy.Subscriber(
            "/casper_auto/current_pose", PoseStamped, self.on_current_pose, queue_size=10)

        '''
        Loop
        '''
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            request = GetNextTrafficLightRequest()
            request.current_pose = self.current_pose
            state, stop_line_location = self.get_tf_light(request)
            self.next_tf_light['state'] = state
            self.next_tf_light['stop_line_location'] = stop_line_location
            self.publish_next_traffic_light_marker()
            rate.sleep()

    def on_current_pose(self, msg):
        self.current_pose = msg.pose
        # print(self.current_pose)

    def get_tf_light(self, request):
        response = roscomp.get_service_response(GetNextTrafficLight)
        try:
            response = self.get_tf_light_service.call(request)
        except:
            pass
        state = response.state
        stop_line_location = response.stop_line
        print(state)
        print(stop_line_location)
        return state, stop_line_location

    def create_tf_light_marker(self, tf_light):
        tf_light_marker = Marker()

        state = tf_light['state']
        stop_line_location = tf_light['stop_line_location']

        tf_light_marker.header.frame_id = "map"
        # tf_light_marker.header.stamp = rospy.Time.now()
        tf_light_marker.ns = "next_tf_light_marker"
        tf_light_marker.type = Marker.CUBE
        tf_light_marker.action = Marker.ADD
        tf_light_marker.id = 1
        tf_light_marker.pose.position.x = stop_line_location.x
        tf_light_marker.pose.position.y = stop_line_location.y
        tf_light_marker.pose.position.z = stop_line_location.z
        tf_light_marker.pose.orientation.x = self.current_pose.orientation.x
        tf_light_marker.pose.orientation.y = self.current_pose.orientation.y
        tf_light_marker.pose.orientation.z = self.current_pose.orientation.z
        tf_light_marker.pose.orientation.w = self.current_pose.orientation.w

        tf_light_marker.scale.x = 1.0
        tf_light_marker.scale.y = 10.0
        tf_light_marker.scale.z = 2.5

        # uint8 RED=0
        # uint8 YELLOW=1
        # uint8 GREEN=2
        # uint8 OFF=3
        # uint8 UNKNOWN=4
        if state == 0:
            tf_light_marker.color = self.RED
        elif state == 1:
            tf_light_marker.color = self.YELLOW
        elif state == 2:
            tf_light_marker.color = self.GREEN
        else:
            tf_light_marker.color = self.WHITE

        tf_light_marker.color.a = 0.25

        return tf_light_marker

    def publish_next_traffic_light_marker(self):
        light_marker = self.create_tf_light_marker(self.next_tf_light)
        self.next_tf_light_marker_publisher.publish(light_marker)


def main():
    """
    main function
    """
    rospy.init_node("carla_route_planner", anonymous=True)
    tf_light_test = TfLightTest()
    rospy.loginfo("Done")


if __name__ == "__main__":
    main()
