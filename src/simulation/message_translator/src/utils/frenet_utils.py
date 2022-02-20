#!/usr/bin/env python

########################################################################
# frenet utils
########################################################################

import math
import numpy as np
import bisect
from collections import namedtuple

Point2D = namedtuple('Point2D', ['x', 'y'])

def global_to_local(ref_orig, orientation, p):
    delta = Point2D(p.x - ref_orig.x, p.y - ref_orig.y)

    s = math.sin(-orientation)
    c = math.cos(-orientation)

    out = Point2D(delta.x * c - delta.y * s,
    delta.x * s + delta.y * c)

    return out

def local_to_global(center, theta, p):

    s = math.sin(theta)
    c = math.cos(theta)

    out = Point2D(p.x * c - p.y * s + center.x, p.x * s + p.y * c + center.y)

    return out

def path_to_list(nav_path):
    path_list = []
    distance_acum = 0.0
    s_map = []
    prev_p = None
    for pose in nav_path.poses:
        x = pose.pose.position.x
        y = pose.pose.position.y
        path_list.append(Point2D(x, y))
        if prev_p != None:
            distance_acum += distance(prev_p.x, prev_p.y, x, y)
        s_map.append(distance_acum)
        prev_p = Point2D(x, y)
    return path_list, s_map

def distance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))

def closest_point_ind(path, x, y):
    index = 0
    closest_index = 0
    min_dist = 10000.0
    for p in path:
        dist = distance(p.x, p.y, x, y)
        if dist < min_dist:
            min_dist = dist
            closest_index = index
        index += 1
    return closest_index

# Transform from Cartesian x,y coordinates to Frenet s,d coordinates
def get_frenet(x, y, path, s_map):
    if path == None:
        print("Empty map. Cannot return Frenet coordinates")
        return 0.0, 0.0, False

    ind_closest = closest_point_ind(path, x, y)

    # Determine the indices of the 2 closest points
    if ind_closest < len(path):
        # Check if we are at the end of the segment
        if ind_closest == len(path) - 1:
            use_previous = True
        elif ind_closest == 0:
            use_previous = False
        else:
            dist_prev = distance(path[ind_closest-1].x, path[ind_closest-1].y, x, y)
            dist_next = distance(path[ind_closest+1].x, path[ind_closest+1].y, x, y)

            if dist_prev <= dist_next:
                use_previous = True
            else:
                use_previous = False

        # Get the 2 points
        if use_previous:
            p1 = Point2D(path[ind_closest - 1].x, path[ind_closest - 1].y)
            p2 = Point2D(path[ind_closest].x, path[ind_closest].y)
            prev_idx = ind_closest - 1
        else:
            p1 = Point2D(path[ind_closest].x, path[ind_closest].y)
            p2 = Point2D(path[ind_closest + 1].x, path[ind_closest + 1].y)
            prev_idx = ind_closest

        # Get the point in the local coordinate with center p1
        theta = math.atan2(p2.y - p1.y, p2.x - p1.x)
        local_p = global_to_local(p1, theta, Point2D(x,y))

        # Get the coordinates in the Frenet frame
        p_s = s_map[prev_idx] + local_p.x
        p_d = local_p.y

    else:
        print("Incorrect index")
        return 0.0, 0.0, False

    return p_s, p_d, True

def get_frenet_with_theta(x, y, path, s_map):
    if path == None:
        print("Empty map. Cannot return Frenet coordinates")
        return 0.0, 0.0, 0.0, False

    ind_closest = closest_point_ind(path, x, y)

    # Determine the indices of the 2 closest points
    if ind_closest < len(path):
        # Check if we are at the end of the segment
        if ind_closest == len(path) - 1:
            use_previous = True
        elif ind_closest == 0:
            use_previous = False
        else:
            dist_prev = distance(path[ind_closest-1].x, path[ind_closest-1].y, x, y)
            dist_next = distance(path[ind_closest+1].x, path[ind_closest+1].y, x, y)

            if dist_prev <= dist_next:
                use_previous = True
            else:
                use_previous = False

        # Get the 2 points
        if use_previous:
            p1 = Point2D(path[ind_closest - 1].x, path[ind_closest - 1].y)
            p2 = Point2D(path[ind_closest].x, path[ind_closest].y)
            prev_idx = ind_closest - 1
        else:
            p1 = Point2D(path[ind_closest].x, path[ind_closest].y)
            p2 = Point2D(path[ind_closest + 1].x, path[ind_closest + 1].y)
            prev_idx = ind_closest

        # Get the point in the local coordinate with center p1
        theta = math.atan2(p2.y - p1.y, p2.x - p1.x)
        local_p = global_to_local(p1, theta, Point2D(x,y))

        # Get the coordinates in the Frenet frame
        p_s = s_map[prev_idx] + local_p.x
        p_d = local_p.y

    else:
        print("Incorrect index")
        return 0.0, 0.0, 0.0, False

    return p_s, p_d, theta, True

# Transform from Frenet s,d coordinates to Cartesian x,y
def get_xy(s, d, path, s_map):

    if path == None or s_map == None:
        print("Empty path. Cannot compute Cartesian coordinates")
        return 0.0, 0.0, False

    # If the value is out of the actual path send a warning
    if s < 0.0 or s > s_map[-1]:
        if s < 0.0:
            prev_point = 0
        else:
            prev_point = len(s_map) -2
    else:
        # Find the previous point
        idx = bisect.bisect_left(s_map, s)
        prev_point = idx - 1

    p1 = path[prev_point]
    p2 = path[prev_point + 1]

    # Transform from local to global
    theta = math.atan2(p2.y - p1.y, p2.x - p1.x)
    p_xy = local_to_global(p1, theta, Point2D(s - s_map[prev_point], d))

    return p_xy.x, p_xy.y, True

############################################################

def dist_to_line(p1, p2, p):
    p1 = np.array(p1)
    p2 = np.array(p2)
    p = np.array(p)
    d = np.cross(p2-p1,p-p1)/np.linalg.norm(p2-p1)
    return abs(d)
