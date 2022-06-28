"""
    This file contains the functions and data types to facilitate
    calculations that are used in the control of the Turtlebot in
    'turtle.py'
"""

from geometry_msgs.msg import Point, Pose
import math

def pose_to_pt(pose: Pose) -> Point:
    """ Convert a Pose object to a Point object """
    return Point(x=pose.position.x, y=pose.position.y, z=pose.position.z)

def pt_to_pose(pt: Point) -> Pose:
    """ Convert a Point object into a Pose object """
    return Pose(x=pt.x, y=pt.y, z=pt.z)

def dist(p1: Point, p2: Point) -> float:
    """ Calculate distance between two points """
    return math.sqrt(((p1.x - p2.x) ** 2 + 
                      (p1.y - p2.y) ** 2 + 
                      (p1.z - p2.z) ** 2))

def reference_potential_grad(ref: Point, cur: Point) -> Point:
    """ Gradient of potential between reference point and current point """
    return Point(x=2 * (cur.x - ref.x), 
                 y=2 * (cur.y - ref.y), 
                 z=2 * (cur.z - ref.z))

def repulsive_potential_grad(p1: Point, p2: Point) -> Point:
    """ Gradient of so-called "repulsive potential" between p1 and p2 """
    return Point(x=(-2 * (p1.x - p2.x) / (dist(p1, p2) ** 2)),
                 y=(-2 * (p1.y - p2.y) / (dist(p1, p2) ** 2)),
                 z=(-2 * (p1.z - p2.z) / (dist(p1, p2) ** 2)))

def add_vecs(v1: Point, v2: Point) -> Point:
    """ Return the addition of two Points, which act as vectors """
    return Point(x=(v1.x + v2.x),
                 y=(v1.y + v2.y),
                 z=(v1.z + v2.z))

def normalize(theta: float) -> float:
    """ Return angle expressed b/t 0 and 2pi """
    if 0.0 <= theta < 2 * math.pi:
        return theta
    elif theta >= 2 * math.pi:
        return theta - math.floor(theta / 2 / math.pi) * 2 * math.pi
    elif theta < 0.0:
        return -theta + math.floor(-theta / 2 / math.pi) * 2 * math.pi