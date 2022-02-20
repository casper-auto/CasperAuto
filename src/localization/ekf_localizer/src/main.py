#!/usr/bin/env python
import math
import numpy as np
import pymap3d as pm
import xml.etree.ElementTree as ET

import pickle
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rotations

import rospy
# Because of transformations
import tf
import tf2_ros
from std_msgs.msg import Header, String, Float32
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu

"""
class EsEkfLocalizer
"""
class EsEkfLocalizer(object):

    def __init__(self):

        '''
        Load parameters
        '''
        # self.val = rospy.get_param("~param", default_val)

        '''
        Constants
        '''
        self.var_imu_f = 0.01
        self.var_imu_w = 0.01
        self.var_gnss = 0.1
        self.var_lidar = 35
        self.gravity = 9.81

        self.g = np.array([0, 0, -self.gravity])  # gravity
        self.l_jac = np.zeros([9, 6])
        self.l_jac[3:, :] = np.eye(6)  # motion model noise jacobian
        self.h_jac = np.zeros([3, 9])
        self.h_jac[:, :3] = np.eye(3)  # measurement model jacobian

        '''
        Variables
        '''
        self.p_est = None
        self.v_est = None
        self.q_est = None
        self.p_cov = None

        self.gnss_data = None
        self.imu_data = None
        self.lidar_data = None

        self.current_position = Point()
        self.current_yaw = 0.0
        self.current_speed = 0.0
        self.opendrive_str = ""
        self.geo_reference = [0.0, 0.0, 0.0]

        self.current_pose = Pose()
        self.filtered_pose = Pose()

        '''

        '''
        self.broadcaster = tf2_ros.TransformBroadcaster()

        '''
        Publishers
        '''
        self.current_pose_pub = rospy.Publisher('/current_pose/filtered', PoseStamped, queue_size=10)
        self.ego_odom_pub = rospy.Publisher('/odometry/filtered', Odometry, queue_size=10)

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

    def run(self):

        if self.imu_data:
            delta_t = 0.1 # compute using acutual ros timestamp

            # Update nominal state with IMU inputs
            Rotation_Mat = rotation.Quaternion(*self.q_est).to_mat()
            self.p_est = self.p_est + delta_t * self.v_est + 0.5 * (delta_t ** 2) * (Rotation_Mat.dot(imu_data[3:]) + g)
            self.v_est = self.v_est + delta_t * (Rotation_Mat.dot(imu_data[3:]) - g)
            self.q_est = rotation.Quaternion(euler = delta_t * imu_data[:3]).quat_mult(self.q_est)

            # Linearize Motion Model and compute Jacobians
            F = np.eye(9)
            imu = imu_data[3:].reshape((3, 1))
            F[0:3, 3:6] = delta_t * np.eye(3)
            F[3:6, 6:9] = Rotation_Mat.dot(-rotation.skew_symmetric(imu)) * delta_t

            # Propagate uncertainty
            Q = np.eye(6)
            Q[0:3, 0:3] = self.var_imu_f * Q[0:3, 0:3]
            Q[3:6, 3:6] = self.var_imu_w * Q[3:6, 3:6]
            Q = (delta_t ** 2) * Q #Integration acceleration to obstain Position
            self.p_cov = F.dot(self.p_cov).dot(F.T) + self.l_jac.dot(Q).dot(self.l_jac.T)

            # GNSS measurement
            if self.gnss_data:
                self.p_est, self.v_est, self.q_est, self.p_cov = self.measurement_update(self.var_gnss, self.p_cov, self.gnss_data, self.p_est, self.v_est, self.q_est)

            # Lidar measurement
            if self.lidar_data:
                self.p_est, self.v_est, self.q_est, self.p_cov = self.measurement_update(self.var_lidar, self.p_cov, self.lidar_data, self.p_est, self.v_est, self.q_est)

            self.filtered_pose = Pose()
            self.filtered_pose.position = Point(*self.p_est)
            quat = tf.transformations.quaternion_from_euler(0, 0, self.current_yaw)
            self.filtered_pose.orientation.x = quat[0]
            self.filtered_pose.orientation.y = quat[1]
            self.filtered_pose.orientation.z = quat[2]
            self.filtered_pose.orientation.w = quat[3]

            pose = PoseStamped()
            pose.header.stamp = rospy.get_rostime()
            pose.header.frame_id = "map"
            pose.pose = self.filtered_pose
            self.current_pose_pub.publish(pose)

            odom = Odometry()
            odom.header.stamp = rospy.get_rostime()
            odom.header.frame_id = "map"
            odom.child_frame_id = "odom"
            odom.pose.pose = self.filtered_pose
            odom.twist.twist.linear.x = self.current_speed
            self.ego_odom_pub.publish(odom)

    """
    Callback functions
    """
    def gnss_callback(self, msg):
        # rospy.loginfo("Received")
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        self.current_position = self.convert_to_local_position([lat, lon, alt])

        self.gnss_data = np.zeros(3)
        self.gnss_data[0] = self.current_position.x
        self.gnss_data[1] = self.current_position.y
        self.gnss_data[2] = self.current_position.z

    def imu_callback(self, msg):
        # rospy.loginfo("Received")
        quat = msg.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.current_yaw = yaw + math.pi/2.0

        self.imu_data = np.zeros(6)
        self.imu_data[0] = msg.angular_velocity.x
        self.imu_data[1] = msg.angular_velocity.y
        self.imu_data[2] = msg.angular_velocity.z
        self.imu_data[3] = msg.linear_acceleration.x
        self.imu_data[4] = msg.linear_acceleration.y
        self.imu_data[5] = msg.linear_acceleration.z

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

    def measurement_update(self, sensor_var, p_cov_check, y_k, p_check, v_check, q_check):
        '''
        A measurement update function for both the GNSS and the LIDAR data.
        '''
        # 1. Compute Kalman Gain
        R_cov = sensor_var * np.eye(3)
        K = p_cov_check.dot(self.h_jac.T.dot(np.linalg.inv(self.h_jac.dot(p_cov_check.dot(self.h_jac.T)) + R_cov)))

        # 2. Compute error state
        delta_x = K.dot(y_k - p_check)

        # 3. Correct predicted state
        p_check = p_check + delta_x[:3]
        v_check = v_check + delta_x[3:6]
        q_check = rotation.Quaternion(axis_angle=delta_x[6:]).quat_mult(q_check)

        # 3.4 Compute corrected covariance
        p_cov_check = (np.eye(9) - K.dot(self.h_jac)).dot(p_cov_check)

        return p_check, v_check, q_check, p_cov_check

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
        x, y, z = pm.geodetic2enu(geo_loc[0],
                                  geo_loc[1],
                                  geo_loc[2],
                                  self.geo_reference[0],
                                  self.geo_reference[1],
                                  self.geo_reference[2],
                                  ell=pm.utils.Ellipsoid('wgs84'))
        return Point(x, y-1.4, z-2.0)

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
    gnss = EsEkfLocalizer()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        gnss.update()
        rate.sleep()

if __name__ == '__main__':
    main()
