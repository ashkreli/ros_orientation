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
    return math.sqrt((p1.x - p2.x) ** 2 + 
                     (p1.y - p2.y) ** 2 + 
                     (p1.z - p2.z) ** 2)


def euler_from_quaternion(q_x, q_y, q_z, q_w):
    """ Converts a quaternion into Euler angles """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q_w * q_x + q_y * q_z)
    cosr_cosp = 1 - 2 * (q_x * q_x + q_y * q_y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (q_w * q_y - q_z * q_x)
    pitch = None
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q_w * q_z + q_x * q_y)
    cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return (roll, pitch, yaw)


def add_vecs(v1: Point, v2: Point) -> Point:
    """ Return the addition of two Points, which act as vectors """
    return Point(x=(v1.x + v2.x),
                 y=(v1.y + v2.y),
                 z=(v1.z + v2.z))


def pt_equal(p1: Point, p2: Point) -> bool:
    """ Returns whether two Points are mathematically equivalent """
    return p1.x == p2.x and p1.y == p2.y and p1.z == p2.z


def normalize(angle: float) -> float:
    """ Returns angle (radians) between 0 and 2pi """
    while angle < 0.0:
        angle += 2. * math.pi
    while angle >= 2 * math.pi:
        angle -= 2. * math.pi
    return angle


def reference_potential_grad(ref: Point, cur: Point) -> Point:
    """ Gradient of potential between reference point and current point """
    return Point(x=10. * (cur.x - ref.x), 
                 y=10. * (cur.y - ref.y), 
                 z=10. * (cur.z - ref.z))


def repulsive_potential_grad(p1: Point, p2: Point) -> Point:
    """ Gradient of so-called "repulsive potential" between p1 and p2 """
    return Point(x=(-2 * (p1.x - p2.x) / (dist(p1, p2) ** 4)),
                 y=(-2 * (p1.y - p2.y) / (dist(p1, p2) ** 4)),
                 z=0.0)