"""
    This file contains the functions and data types to facilitate
    calculations that are used in the control of the Turtlebot in
    'turtle.py'
"""

from geometry_msgs.msg import Point, Pose
import math
import numpy as np

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

global max_linear_velocity,max_angular_velocity
max_linear_velocity = 1.0 # meters per second
max_angular_velocity = 1.0 # radians per second

def getB(yaw, deltat):
    """
    Calculates and returns the B matrix
    3x2 matix ---> number of states x number of control inputs
 
    Expresses how the state of the system [x,y,yaw] changes
    from t-1 to t due to the control commands (i.e. control inputs).
     
    :param yaw: The yaw angle (rotation angle around the z axis) in radians 
    :param deltat: The change in time from timestep t-1 to t in seconds
     
    :return: B matrix ---> 3x2 NumPy array
    """
    B = np.array([  [np.cos(yaw)*deltat, 0],
                                    [np.sin(yaw)*deltat, 0],
                                    [0, deltat]])
    return B


def lqr(actual_state_x, desired_state_xf, Q, R, A, B, dt):
    """
    Discrete-time linear quadratic regulator for a nonlinear system.
 
    Compute the optimal control inputs given a nonlinear system, cost matrices, 
    current state, and a final state.
     
    Compute the control variables that minimize the cumulative cost.
 
    Solve for P using the dynamic programming method.
 
    :param actual_state_x: The current state of the system 
        3x1 NumPy Array given the state is [x,y,yaw angle] --->
        [meters, meters, radians]
    :param desired_state_xf: The desired state of the system
        3x1 NumPy Array given the state is [x,y,yaw angle] --->
        [meters, meters, radians]   
    :param Q: The state cost matrix
        3x3 NumPy Array
    :param R: The input cost matrix
        2x2 NumPy Array
    :param dt: The size of the timestep in seconds -> float
 
    :return: u_star: Optimal action u for the current state 
        2x1 NumPy Array given the control input vector is
        [linear velocity of the car, angular velocity of the car]
        [meters per second, radians per second]
    """
    # We want the system to stabilize at desired_state_xf.
    x_error = actual_state_x - desired_state_xf
 
    # Solutions to discrete LQR problems are obtained using the dynamic 
    # programming method.
    # The optimal solution is obtained recursively, starting at the last 
    # timestep and working backwards.
    # You can play with this number
    N = 50
 
    # Create a list of N + 1 elements
    P = [None] * (N + 1)
     
    Qf = Q
 
    # LQR via Dynamic Programming
    P[N] = Qf
 
    # For i = N, ..., 1
    for i in range(N, 0, -1):
 
        # Discrete-time Algebraic Riccati equation to calculate the optimal 
        # state cost matrix
        P[i-1] = Q + A.T @ P[i] @ A - (A.T @ P[i] @ B) @ np.linalg.pinv(
            R + B.T @ P[i] @ B) @ (B.T @ P[i] @ A)      
 
    # Create a list of N elements
    K = [None] * N
    u = [None] * N
 
    # For i = 0, ..., N - 1
    for i in range(N):
 
        # Calculate the optimal feedback gain K
        K[i] = -np.linalg.pinv(R + B.T @ P[i+1] @ B) @ B.T @ P[i+1] @ A
 
        u[i] = K[i] @ x_error
 
    # Optimal control input is u_star (within the limits provided)
    linear_velocity = np.clip(u[N-1][0],-max_linear_velocity,max_linear_velocity)
    angular_velocity = np.clip(u[N-1][1],-max_angular_velocity,max_angular_velocity)
    u_star = [linear_velocity,angular_velocity]
 
    return u_star