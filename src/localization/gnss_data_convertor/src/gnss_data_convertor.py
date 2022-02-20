#!/usr/bin/env python
import math
import numpy as np
import pymap3d as pm
import xml.etree.ElementTree as ET

import rospy
# Because of transformations
import tf
import tf2_ros
from std_msgs.msg import Header, String, Float32
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu

"""
class GnssImuLocalizer
"""
class GnssImuLocalizer(object):

    def __init__(self):

        '''
        Load parameters
        '''
        # self.val = rospy.get_param("~param", default_val)

        '''
        Constants
        '''
        # Earth radius at equator [m].
        self.EARTH_RADIUS_EQUA = 6378137.0

        '''
        Variables
        '''
        self.current_position = Point()
        self.current_yaw = 0.0
        self.current_speed = 0.0
        self.opendrive_str = ""
        self.geo_reference = [0.0, 0.0, 0.0]

        self.current_pose = Pose()

        '''
        Broadcaster
        '''
        self.broadcaster = tf2_ros.TransformBroadcaster()

        '''
        Publishers
        '''
        self.current_pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=10)
        self.ego_odom_pub = rospy.Publisher('/odometry', Odometry, queue_size=10)

        '''
        Subscribers
        '''
        self.gnss_subscriber = rospy.Subscriber(
            "/carla/ego_vehicle/gnss/fix", NavSatFix, self.gnss_callback, queue_size=10)
        self.imu_subscriber = rospy.Subscriber(
            "/carla/ego_vehicle/imu", Imu, self.imu_callback, queue_size=10)
        self.speedometer_subscriber = rospy.Subscriber(
            "/carla/ego_vehicle/speedometer", Float32, self.speedometer_callback, queue_size=10)
        self.opendrive_map_subscriber = rospy.Subscriber(
            '/carla/ego_vehicle/opendrive_map', String, self.opendrive_map_callback, queue_size=10)

    """
    Callback functions
    """
    def gnss_callback(self, msg):
        # rospy.loginfo("Received")
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        self.current_position = self.convert_to_local_position([lat, lon, alt])

    def imu_callback(self, msg):
        # rospy.loginfo("Received")
        quat = msg.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.current_yaw = yaw + math.pi/2.0

    def speedometer_callback(self, msg):
        # rospy.loginfo("Received")
        self.current_speed = msg.data

    def opendrive_map_callback(self, msg):
        # rospy.loginfo("Received")
        self.opendrive_str = msg.data
        self.geo_reference = self.get_geo_reference(self.opendrive_str)

    """
    Helper functions
    """

    def lat_to_scale(self, lat):
        '''
        Convert latitude to scale, which is needed by mercator transformations
            @param lat latitude in degrees (DEG)
            @return scale factor
            @note when converting from lat/lon -> mercator and back again,
                  or vice versa, use the same scale in both transformations!
        '''
        return math.cos(np.deg2rad(lat))

    def latlon_to_mercator(self, lat, lon, scale):
        '''
        Converts lat/lon/scale to mx/my (mx/my in meters if correct scale is given).
        template <class float_type>
        '''
        mx = scale * np.deg2rad(lon) * self.EARTH_RADIUS_EQUA;
        my = scale * self.EARTH_RADIUS_EQUA * math.log(math.tan((90.0 + lat) * math.pi / 360.0));
        return mx, my

    def mercator_to_latlon(self, mx, my, scale):
        '''
        Converts mx/my/scale to lat/lon (mx/my in meters if correct scale is given).
        '''
        lon = mx * 180.0 / (math.pi * self.EARTH_RADIUS_EQUA * scale);
        lat = 360.0 * math.atan(math.exp(my / (self.EARTH_RADIUS_EQUA * scale))) / math.pi - 90.0;
        return lon, lat

    def get_geo_reference(self, opendrive_str):
        xml_tree = ET.fromstring(opendrive_str)
        lat_ref, lon_ref = None, None
        for geo_elem in xml_tree.find('header').find('geoReference').text.split(' '):
            if geo_elem.startswith('+lat_0'):
                lat_ref = float(geo_elem.split('=')[-1])
            elif geo_elem.startswith('+lon_0'):
                lon_ref = float(geo_elem.split('=')[-1])
        return lat_ref, lon_ref, 0.

    def convert_to_local_position(self, geo_loc):
        # x, y, z = pm.geodetic2enu(geo_loc[0],
        #                           geo_loc[1],
        #                           geo_loc[2],
        #                           self.geo_reference[0],
        #                           self.geo_reference[1],
        #                           self.geo_reference[2],
        #                           ell=pm.utils.Ellipsoid('wgs84'))

        scale = self.lat_to_scale(self.geo_reference[0])
        x0, y0 = self.latlon_to_mercator(self.geo_reference[0], self.geo_reference[0], scale)
        scale = self.lat_to_scale(geo_loc[0])
        x1, y1 = self.latlon_to_mercator(geo_loc[0], geo_loc[1], scale)

        return Point(x1-x0, y1-y0, 0.0)

    def construct_current_pose(self):
        self.current_pose = Pose()
        self.current_pose.position = self.current_position
        quat = tf.transformations.quaternion_from_euler(0, 0, self.current_yaw)
        self.current_pose.orientation.x = quat[0]
        self.current_pose.orientation.y = quat[1]
        self.current_pose.orientation.z = quat[2]
        self.current_pose.orientation.w = quat[3]

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = rospy.get_rostime()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation = self.current_pose.position
        t.transform.rotation = self.current_pose.orientation
        self.broadcaster.sendTransform(t)

    def publish_current_pose(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = "map"
        pose.pose = self.current_pose
        self.current_pose_pub.publish(pose)

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = rospy.get_rostime()
        odom.header.frame_id = "map"
        odom.child_frame_id = "odom"
        odom.pose.pose = self.current_pose
        odom.twist.twist.linear.x = self.current_speed
        self.ego_odom_pub.publish(odom)

    def update(self):
        self.construct_current_pose()
        self.broadcast_tf()
        self.publish_current_pose()
        self.publish_odometry()

"""
main function
"""
def main():
    rospy.init_node('gnss_imu_localizer', anonymous=True)
    gnss = GnssImuLocalizer()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        gnss.update()
        rate.sleep()

if __name__ == '__main__':
    main()
