#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import time
import carla
import rospy
import math
import random


def get_transform(vehicle_location, angle=90, d=0.0):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 75.0) + vehicle_location
    return carla.Transform(location, carla.Rotation(yaw=angle, pitch=-90))


def main():
    """
    main function
    """
    rospy.init_node('carla_manual_control', anonymous=True)

    rate = rospy.Rate(100) # 10hz

    role_name = rospy.get_param("~role_name", "ego_vehicle")
    host = rospy.get_param("/carla/host", "127.0.0.1")
    port = rospy.get_param("/carla/port", 2000)
    vehicle_filter = rospy.get_param("/vehicle_filter", "vehicle.tesla.*")

    rospy.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(
        host=host, port=port))

    # Wait for a while and let carla get up
    time.sleep(2)

    try:
        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(2)

        rospy.loginfo("Connected to Carla.")

        world = carla_client.get_world()
        spectator = world.get_spectator()

        location = None

        # keep tracking the best view point
        while not rospy.is_shutdown():
            vehicle_list = world.get_actors().filter(vehicle_filter)
            if location is None and len(vehicle_list) > 0:
                ego_vehicle = vehicle_list[0]
                location = ego_vehicle.get_location()
                spectator.set_transform(get_transform(location))
                rospy.loginfo("Keep tracking the best view point.")
            rate.sleep()

    finally:
        pass


if __name__ == '__main__':

    main()
