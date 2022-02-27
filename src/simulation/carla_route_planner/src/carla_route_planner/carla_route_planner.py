"""
Generates a plan of waypoints to follow

It uses the current pose of the ego vehicle as starting point. If the
vehicle is respawned, the route is newly calculated.

The goal is either read from the ROS topic `/carla/<ROLE NAME>/move_base_simple/goal`, if available
(e.g. published by RVIZ via '2D Nav Goal') or a fixed spawnpoint is used.

The calculated route is published on '/carla/<ROLE NAME>/waypoints'
"""
import math
import threading
import random

import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32, ColorRGBA
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from visualization_msgs.msg import Marker, MarkerArray
from carla_msgs.msg import CarlaWorldInfo

import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO


class CarlaRoutePlanner(object):

    """
    This class generates a plan of waypoints to follow.

    The calculation is done whenever:
    - the ego_vehicle vehicle appears
    - a new goal is set
    """
    WAYPOINT_DISTANCE = 2.0

    def __init__(self, carla_world):
        self.world = carla_world
        self.map = carla_world.get_map()
        self.ego_vehicle = None
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')
        self.route_range = float(rospy.get_param('~route_range', '50.0'))
        self.global_route_publisher = rospy.Publisher(
            '/casper_auto/global_route', Path, queue_size=10, latch=True)
        self.global_route_markers_publisher = rospy.Publisher(
            '/casper_auto/global_route_markers', MarkerArray, queue_size=10, latch=True)

        self.actor_spawnpoint = None
        # check argument and set spawn_point
        spawn_point_param = rospy.get_param('~spawn_point')
        if spawn_point_param:
            rospy.loginfo("Using ros parameter for spawnpoint: {}".format(spawn_point_param))
            spawn_point = spawn_point_param.split(',')
            if len(spawn_point) != 6:
                raise ValueError("Invalid spawnpoint '{}'".format(spawn_point_param))
            pose = Pose()
            pose.position.x = float(spawn_point[0])
            pose.position.y = float(spawn_point[1])
            pose.position.z = float(spawn_point[2])
            quat = quaternion_from_euler(
                math.radians(float(spawn_point[3])),
                math.radians(float(spawn_point[4])),
                math.radians(float(spawn_point[5])))
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            self.actor_spawnpoint = pose

        # check argument and set spawn_point
        self.goal_set = False
        target_point_param = rospy.get_param('~target_point')
        if target_point_param:
            rospy.loginfo("Using ros parameter for spawnpoint: {}".format(target_point_param))
            target_point = target_point_param.split(',')
            if len(target_point) != 3:
                raise ValueError("Invalid spawnpoint '{}'".format(target_point_param))
            else:
                self.actor_target_location = carla.Location(float(target_point[0]), -float(target_point[1]), float(target_point[2]))
                self.goal_set = True

        self.start = None # carla.Location()
        self.goal = None
        self.global_route = None
        self.lane_fixed = False

        self.current_pose_subscriber = rospy.Subscriber(
            "/casper_auto/current_pose", PoseStamped, self.on_current_pose, queue_size=10)

        self._update_lock = threading.Lock()

        # use callback to wait for ego vehicle
        rospy.loginfo("Waiting for ego vehicle...")
        self.world.on_tick(self.find_ego_vehicle_actor)

    def on_current_pose(self, msg):
        # rospy.loginfo("Pose: x={}, y={}, z={}".format(
        #               msg.pose.position.x,
        #               msg.pose.position.y,
        #               msg.pose.position.z))

        pose = msg.pose

        if self.actor_spawnpoint:
            pose = self.actor_spawnpoint

        if not self.lane_fixed:
            self.lane_fixed = True

            carla_start = carla.Transform()
            carla_start.location.x = pose.position.x
            carla_start.location.y = -pose.position.y
            carla_start.location.z = pose.position.z + 2  # 2m above ground
            quaternion = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            )

            _, _, yaw = euler_from_quaternion(quaternion)
            carla_start.rotation.yaw = -math.degrees(yaw)

            self.start = carla_start
            self.reroute()

        self.publish_global_route()
        self.publish_global_route_marker()

    def reroute(self):
        """
        Triggers a rerouting
        """
        if self.ego_vehicle is None or self.start is None:
            # no ego vehicle, remove route if published
            self.global_route = None
        else:
            self.global_route = self.calculate_route(self.start)

    def find_ego_vehicle_actor(self, _):
        """
        Look for an carla actor with name 'ego_vehicle'
        """
        with self._update_lock:
            ego_vehicle = None
            for actor in self.world.get_actors():
                if actor.attributes.get('role_name') == self.role_name:
                    ego_vehicle = actor
                    break

            ego_vehicle_changed = False
            if ego_vehicle is None and self.ego_vehicle is not None:
                ego_vehicle_changed = True

            if not ego_vehicle_changed and ego_vehicle is not None and self.ego_vehicle is None:
                ego_vehicle_changed = True

            if not ego_vehicle_changed and ego_vehicle is not None and \
                    self.ego_vehicle is not None and ego_vehicle.id != self.ego_vehicle.id:
                ego_vehicle_changed = True

            if ego_vehicle_changed:
                rospy.loginfo("Ego vehicle changed.")
                self.ego_vehicle = ego_vehicle
                self.reroute()

    def calculate_route(self, start):

        # rospy.loginfo("Calculating route from x={}, y={}, z={}".format(
        #     start.location.x,
        #     start.location.y,
        #     start.location.z))

        """
        Calculate the route
        """
        start_location = carla.Location(start.location.x,
                                        start.location.y,
                                        start.location.z)

        # start_location = self.ego_vehicle.get_location()

        start_waypoint = self.map.get_waypoint(start_location)

        if self.goal_set:
            goal_location = self.actor_target_location
        else:
            goal_waypoint = random.choice(start_waypoint.next(self.route_range))
            goal = goal_waypoint.transform
            goal_location = carla.Location(goal.location.x,
                                           goal.location.y,
                                           goal.location.z)

        dao = GlobalRoutePlannerDAO(self.map, sampling_resolution=1.0)
        grp = GlobalRoutePlanner(dao)
        grp.setup()
        lane_keep_route = grp.trace_route(start_location, goal_location)

        return lane_keep_route

    def publish_global_route(self):
        """
        Publish the ROS message containing the waypoints
        """
        path = Path()
        path.header.frame_id = "map"
        # path.header.stamp = rospy.Time.now()
        if self.global_route is not None:
            for wp in self.global_route:
                pose = PoseStamped()
                pose.pose.position.x = wp[0].transform.location.x
                pose.pose.position.y = -wp[0].transform.location.y
                pose.pose.position.z = wp[0].transform.location.z

                quaternion = tf.transformations.quaternion_from_euler(
                    0, 0, -math.radians(wp[0].transform.rotation.yaw))
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                path.poses.append(pose)

        self.global_route_publisher.publish(path)
        # rospy.loginfo("Published {} waypoints in the global route.".format(len(path.poses)))

    def create_global_waypoint_array_marker(self, color):
        waypoint_marker = Marker()

        waypoint_marker.header.frame_id = "map"
        # waypoint_marker.header.stamp = rospy.Time.now()
        waypoint_marker.ns = "global_route_marker"
        waypoint_marker.type = Marker.LINE_STRIP
        waypoint_marker.action = Marker.ADD
        waypoint_marker.scale.x = 0.75
        waypoint_marker.scale.y = 0.75
        waypoint_marker.color = color
        waypoint_marker.frame_locked = False

        waypoint_marker.id = 1
        waypoint_marker.pose.orientation.w = 1.0

        if self.global_route is not None:
            for wp in self.global_route:
                point = Point()
                point.x = wp[0].transform.location.x
                point.y = -wp[0].transform.location.y
                point.z = wp[0].transform.location.z
                waypoint_marker.points.append(point)

        return waypoint_marker

    def create_global_waypoint_array_orientation_marker(self):
        marker_array = MarkerArray()

        if self.global_route is not None:
            count = 1
            for wp in self.global_route:
                pose = Pose()
                pose.position.x = wp[0].transform.location.x
                pose.position.y = -wp[0].transform.location.y
                pose.position.z = wp[0].transform.location.z
                quaternion = tf.transformations.quaternion_from_euler(
                    0, 0, -math.radians(wp[0].transform.rotation.yaw))
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]

                waypoint_marker = Marker()
                waypoint_marker.header.frame_id = "map"
                # waypoint_marker.header.stamp = rospy.Time.now()
                waypoint_marker.type = Marker.ARROW
                waypoint_marker.action = Marker.ADD
                waypoint_marker.scale.x = 0.6
                waypoint_marker.scale.y = 0.2
                waypoint_marker.scale.z = 0.1
                waypoint_marker.color.r = 1.0
                waypoint_marker.color.a = 1.0
                # waypoint_marker.frame_locked = False
                waypoint_marker.ns = "global_route_orientation_marker"
                waypoint_marker.id = count
                waypoint_marker.pose = pose
                waypoint_marker.color.r = 0.0
                waypoint_marker.color.g = 1.0
                waypoint_marker.color.b = 0.0
                marker_array.markers.append(waypoint_marker)

                count += 1

        return marker_array

    def publish_global_route_marker(self):
        total_color = ColorRGBA()
        total_color.r = 0
        total_color.g = 0.7
        total_color.b = 1.0
        total_color.a = 0.9

        marker_array = MarkerArray()
        marker = self.create_global_waypoint_array_marker(total_color)
        temp_array = self.create_global_waypoint_array_orientation_marker()
        marker_array.markers.append(marker)
        for mk in temp_array.markers:
            marker_array.markers.append(mk)

        self.global_route_markers_publisher.publish(marker_array)


def main():
    """
    main function
    """
    rospy.init_node("carla_route_planner", anonymous=True)

    # wait for ros-bridge to set up CARLA world
    rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
    try:
        rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
    except rospy.ROSException as e:
        rospy.logerr("Timeout while waiting for world info!")
        raise e

    host = rospy.get_param("/carla/host", "127.0.0.1")
    port = rospy.get_param("/carla/port", 2000)

    rospy.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(
        host=host, port=port))

    try:
        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(2)

        carla_world = carla_client.get_world()

        rospy.loginfo("Connected to Carla.")

        waypointConverter = CarlaRoutePlanner(carla_world)

        rospy.spin()
        del waypointConverter
        del carla_world
        del carla_client

    finally:
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
