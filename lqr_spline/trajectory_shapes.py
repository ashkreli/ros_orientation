#!/usr/bin/env python3

import math
import numpy as np
from typing import Tuple, List

global LIN_VEL, ANG_VEL
LIN_VEL = 1
ANG_VEL = 0.2

def straight_line(pos_start: Tuple[float, float], 
                  pos_end: Tuple[float, float], 
                  N: int) -> Tuple[List[np.array], List[np.array]]:
    """ Returns the ideal s(t) and u(t) for traversing straight line 
        in N time steps"""
    x_start, y_start = pos_start
    x_end, y_end = pos_end
    x_step = (x_end - x_start) / N
    y_step = (y_end - y_start) / N

    theta = math.atan2(y_end - y_start, x_end - x_start)

    s_t = []
    u_t = []

    x = x_start
    y = y_start

    for i in range(0, N):
        s_t.append((np.array([[x], [y], [theta]])))
        u_t.append((np.array([[LIN_VEL], [0.0]])))
        x += x_step
        y += y_step
    
    return s_t, u_t