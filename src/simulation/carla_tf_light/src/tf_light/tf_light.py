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
from carla_tf_light.srv import GetNextTrafficLight, GetNextTrafficLightResponse

import carla
import carla_common.transforms as trans
import ros_compatibility as roscomp


def rotate_point(point, radians):
    """
    rotate a given point by a given angle
    """
    rotated_x = math.cos(radians) * point.x - math.sin(radians) * point.y
    rotated_y = math.sin(radians) * point.x + math.cos(radians) * point.y

    return Point(rotated_x, rotated_y, point.z)

def global_to_local(ref_orig, orientation, p):
    delta = Point(p.x - ref_orig.x, p.y - ref_orig.y, 0)

    s = math.sin(-orientation)
    c = math.cos(-orientation)

    out = Point(delta.x * c - delta.y * s, delta.x * s + delta.y * c, p.z)

    return out

class TfLightPublisher(object):

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
        self.traffic_lights = {}

        '''
        ROS Subscribers
        '''
        self.tf_light_info_subscriber = rospy.Subscriber(
            "/carla/traffic_lights/info", CarlaTrafficLightInfoList, self.on_tf_light_info, queue_size=10)
        self.tf_light_status_subscriber = rospy.Subscriber(
            "/carla/traffic_lights/status", CarlaTrafficLightStatusList, self.on_tf_light_status, queue_size=10)

        '''
        ROS Publishers
        '''
        self.tf_light_markers_publisher = rospy.Publisher(
            '/casper_auto/traffic_lights_markers', MarkerArray, queue_size=10)
        self.tf_light_volume_markers_publisher = rospy.Publisher(
            '/casper_auto/traffic_light_volumes_markers', MarkerArray, queue_size=10)

        '''
        ROS Service
        '''
        self.get_waypoint_service = rospy.Service(
            '/casper_auto/next_tf_light',
            GetNextTrafficLight,
            self.get_next_tf_light)

        '''
        Loop
        '''
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.publish_all_traffic_lights_markers()
            self.publish_all_traffic_lights_volumes_markers()
            rate.sleep()

    def on_tf_light_info(self, msg):
        '''
        carla_msgs/CarlaTrafficLightInfoList
        '''
        for tf_light in msg.traffic_lights:
            if not tf_light.id in self.traffic_lights:
                transform = tf_light.transform  # geometry_msgs/Pose transform
                trigger_volume = tf_light.trigger_volume  # relative to transform
                self.traffic_lights[tf_light.id] = \
                    {'id': tf_light.id,
                     'transform': transform,
                     'trigger_volume': trigger_volume,
                     'state': 0}

        # print(self.traffic_lights)

    def on_tf_light_status(self, msg):
        '''
        carla_msgs/CarlaTrafficLightStatusList
        '''
        for tf_light in msg.traffic_lights:
            if tf_light.id in self.traffic_lights:
                state = tf_light.state
                self.traffic_lights[tf_light.id]['state'] = int(state)

        # print(self.traffic_lights)

    def distance(self, p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        return dx**2 + dy**2

    def get_next_tf_light(self, req, response=None):

        print('Get next tf light service is called.')

        response = GetNextTrafficLightResponse()
        response.state = 4

        point = req.current_pose.position
        quat = req.current_pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]
        _, _, yaw = euler_from_quaternion(quat)

        closest_dist = 10000

        for key in self.traffic_lights:
            tf_light = self.traffic_lights[key]
            trigger_point = self.get_trafficlight_trigger_location(tf_light)
            dist = self.distance(point, trigger_point)

            trigger_point_local = global_to_local(point, yaw, trigger_point)

            if trigger_point_local.x > 0 and trigger_point_local.x < 50.0 \
                and abs(trigger_point_local.y) < 5.0:

                if dist < closest_dist:
                    closest_dist = dist
                    response.state = tf_light['state']
                    response.stop_line = trigger_point

        return response

    def create_tf_light_marker(self, tf_light, height_shift=2.0):
        tf_light_marker = Marker()

        id = int(tf_light['id'])
        transform = tf_light['transform']
        state = tf_light['state']

        tf_light_marker.header.frame_id = "map"
        # tf_light_marker.header.stamp = rospy.Time.now()
        tf_light_marker.ns = "tf_light_marker"
        tf_light_marker.type = Marker.SPHERE
        tf_light_marker.action = Marker.ADD
        tf_light_marker.id = id
        tf_light_marker.pose.position.x = transform.position.x
        tf_light_marker.pose.position.y = transform.position.y
        tf_light_marker.pose.position.z = transform.position.z + height_shift
        tf_light_marker.pose.orientation.x = transform.orientation.x
        tf_light_marker.pose.orientation.y = transform.orientation.y
        tf_light_marker.pose.orientation.z = transform.orientation.z
        tf_light_marker.pose.orientation.w = transform.orientation.w

        tf_light_marker.scale.x = 0.75
        tf_light_marker.scale.y = 0.75
        tf_light_marker.scale.z = 0.75

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

        return tf_light_marker

    def get_trafficlight_trigger_location(self, tf_light, backward_shitf=5.0):
        base_transform = trans.ros_pose_to_carla_transform(tf_light['transform'])
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(
            trans.ros_point_to_carla_location(tf_light['trigger_volume'].center))
        area_ext = tf_light['trigger_volume'].size

        point = rotate_point(Point(0.0, -backward_shitf, area_ext.z / 2.0), math.radians(base_rot))
        point_location = area_loc + carla.Location(x=point.x, y=point.y)

        return Point(point_location.x, -point_location.y, point_location.z)

    def create_tf_light_bouding_box(self, tf_light):
        bounding_box_marker = Marker()

        id = int(tf_light['id'])
        transform = tf_light['transform']
        volume = tf_light['trigger_volume']
        state = tf_light['state']
        trigger_location = self.get_trafficlight_trigger_location(tf_light)

        bounding_box_marker.header.frame_id = "map"
        # bounding_box_marker.header.stamp = rospy.Time.now()
        bounding_box_marker.ns = "tf_light_volume_marker"
        bounding_box_marker.type = Marker.CUBE
        bounding_box_marker.action = Marker.ADD
        bounding_box_marker.id = id

        bounding_box_marker.pose.position.x = trigger_location.x
        bounding_box_marker.pose.position.y = trigger_location.y
        bounding_box_marker.pose.position.z = trigger_location.z
        bounding_box_marker.pose.orientation.x = transform.orientation.x
        bounding_box_marker.pose.orientation.y = transform.orientation.y
        bounding_box_marker.pose.orientation.z = transform.orientation.z
        bounding_box_marker.pose.orientation.w = transform.orientation.w

        bounding_box_marker.scale.x = volume.size.x
        bounding_box_marker.scale.y = volume.size.y
        bounding_box_marker.scale.z = volume.size.z

        # uint8 RED=0
        # uint8 YELLOW=1
        # uint8 GREEN=2
        # uint8 OFF=3
        # uint8 UNKNOWN=4
        if state == 0:
            bounding_box_marker.color = self.RED
        elif state == 1:
            bounding_box_marker.color = self.YELLOW
        elif state == 2:
            bounding_box_marker.color = self.GREEN
        else:
            bounding_box_marker.color = self.WHITE

        return bounding_box_marker

    def publish_all_traffic_lights_markers(self):
        tf_light_marker_array = MarkerArray()
        for key in self.traffic_lights:
            tf_light = self.traffic_lights[key]
            light_marker = self.create_tf_light_marker(tf_light)
            tf_light_marker_array.markers.append(light_marker)
        self.tf_light_markers_publisher.publish(tf_light_marker_array)

    def publish_all_traffic_lights_volumes_markers(self):
        bounding_box_marker_array = MarkerArray()
        for key in self.traffic_lights:
            tf_light = self.traffic_lights[key]
            volume_marker = self.create_tf_light_bouding_box(tf_light)
            bounding_box_marker_array.markers.append(volume_marker)
        self.tf_light_volume_markers_publisher.publish(bounding_box_marker_array)


def main():
    """
    main function
    """
    rospy.init_node("carla_route_planner", anonymous=True)
    tf_light_publisher = TfLightPublisher()
    rospy.loginfo("Done")


if __name__ == "__main__":
    main()
