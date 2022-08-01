import cvxpy as cp
import numpy as np
import math
from typing import Tuple, List
import matplotlib.pyplot as plt

import sys, pathlib, os
path = pathlib.Path(__file__).parent.parent.absolute()
sys.path.append(os.path.join(str(path), 'utils'))
import sim_move, lqr, calcs


def objective_function(pts: List[Tuple[float, np.array]]):
    """ Set up least-squares linear system """
    # A matrix for a specific waypoint
    def make_A_p(point: Tuple[float, np.array]):
        p = point[0]
        A_p = np.array([[1, p, p**2, p**3, 0, 0, 0, 0],
                        [0, 0, 0, 0, 1, p, p**2, p**3]])
        return A_p
    A = make_A_p(pts[0])
    b = pts[0][1]
    # Stack A and b for each point that is to be solved later as
    # one big system
    for i in range(1, len(pts)):
        A = np.concatenate((A, make_A_p(pts[i])), axis=0)
        b = np.concatenate((b, pts[i][1]), axis=0)
    return A, b

def find_a_b(pts: List[Tuple[float, np.array]]):
    """ Solve least-squares fitting for x(t) and y(t) coeffs """
    a_b = cp.Variable((8, 1))
    A, b = objective_function(pts)
    dist = cp.sum_squares(A @ a_b - b)
    prob = cp.Problem(cp.Minimize(dist))
    prob.solve()

    return a_b.value

def atan2(y: List[float], x: List[float]) -> List[float]:
    """ Returns atan2 of each (x, y) pair """
    a = []
    for i in range(len(y)):
        a.append(math.atan2(y[i], x[i]))
    return a

def sqrt(lst: List[float]) -> List[float]:
    """ Returns the sqrt of each element of a list """
    a = []
    for el in lst:
        a.append(math.sqrt(el))
    return a

def normalize(lst: List[float]) -> List[float]:
    a = []
    for el in lst:
        a.append(calcs.normalize(el))
    return a

def gen_s_u(waypts: List[Tuple[float, np.array]], 
            N: int) -> Tuple[List[np.array], List[np.array]]:
    """ Interpolates 3rd-degree polynomials from given waypoints (t, (x, y)) """
    a_b = find_a_b(waypts)
    # Pull out timespan
    t = np.linspace(waypts[0][0], waypts[-1][0], num=N)
    # x(p) coeffs
    a = a_b[0:4]
    # y(p) coeffs
    b = a_b[4:]
    # x(t) and y(t)
    x = a[0] + a[1] * t + a[2] * t**2 + a[3] * t**3
    y = b[0] + b[1] * t + b[2] * t**2 + b[3] * t**3
    # First time derivatives of x(t) and y(t)
    x_dot = a[1] + 2 * a[2] * t + 3 * a[3] * t**2
    y_dot = b[1] + 2 * b[2] * t + 3 * b[3] * t**2
    #print(str(x_dot))
    theta = atan2(y_dot, x_dot)
    # Second time derivatives - use multiplication rule
    x_dotdot = 2 * a[2] + 6 * a[3] * t
    y_dotdot = 2 * b[2] + 6 * b[3] * t
    # linear and angular velocity ideal inputs
    v = sqrt(x_dot**2 + y_dot**2)
    omega = -x_dotdot * y_dot / v + y_dotdot * x_dot / v
    s_t = []
    u_t = []
    for i in range(len(x)):
        s_t.append(np.array([[x[i]], [y[i]], [theta[i]]]))
        u_t.append(np.array([[v[i]], [omega[i]]]))
    # print(str(u_t))
    return s_t, u_t

def attach_t(waypts: List[np.array], 
             max_vels: List[float]) -> List[Tuple[float, np.array]]:
    """ Estimate time by which each waypoint could be traversed """
    # Make sure max_vel is specified between every two consecutive waypoints
    assert(len(waypts) - 1 <= len(max_vels))
    def dist(p1: np.array, p2: np.array):
        return math.sqrt((p1[0] - p2[0])**2 +
                         (p1[1] - p2[1])**2)
    t = 0.0
    t_attached_waypts = []
    for i in range(len(waypts)):
        t_attached_waypts.append((t, waypts[i]))
        if i != len(waypts) - 1:
            t += dist(waypts[i], waypts[i+1]) / max_vels[i]
    return t_attached_waypts