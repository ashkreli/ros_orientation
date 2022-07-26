#!/usr/bin/env python3

import math
import numpy as np
from typing import Tuple, List


def dist(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """ Returns the distance between p1 and p2 """
    return math.sqrt((p1[0] - p2[0])**2 +
                     (p1[1] - p2[1])**2)


def straight_line(pos_start: Tuple[float, float], 
                  pos_end: Tuple[float, float], 
                  N: int) -> Tuple[List[np.array], List[np.array]]:
    """ Returns the ideal s(t) and u(t) for traversing straight line 
        in N time steps"""
    x_start, y_start = pos_start
    x_end, y_end = pos_end
    x_step = (x_end - x_start) / N
    y_step = (y_end - y_start) / N

    lin_vel = dist(pos_start, pos_end) / N

    theta = math.atan2(y_end - y_start, x_end - x_start)

    s_t = []
    u_t = []

    x = x_start
    y = y_start

    for i in range(0, N):
        s_t.append(np.array([[x], [y], [theta]]))
        u_t.append(np.array([[lin_vel], [0.0]]))
        x += x_step
        y += y_step
    
    return s_t, u_t

def circle(center: Tuple[float, float], 
           radius: float, 
           N     : int,
           loops : int) -> Tuple[List[np.array], List[np.array]]:
    """ Produce CCW circle trajectory over N steps
        Valid only if radius and N are greater than 0 """
    ang_vel = 2 * math.pi / N
    s_t = []
    u_t = []
    
    for a in range(loops):
        x = radius + center[0]
        y = center[1]
        theta = 0
        for i in range(N):
            s_t.append(np.array([[x], [y], [theta + math.pi / 2]]))
            u_t.append(np.array([[ang_vel * radius], [ang_vel]]))
            theta += ang_vel
            x = radius * math.cos(theta) + center[0]
            y = radius * math.sin(theta) + center[1]
    
    return s_t, u_t