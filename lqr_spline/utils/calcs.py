import math
from typing import Tuple
import numpy as np



def euler_from_quaternion(q_x: float, 
                          q_y: float, 
                          q_z: float, 
                          q_w: float) -> Tuple[float, float, float]:
    ''' Converts Quaternion into Euler angles, obtained from:
        https://handwiki.org/wiki/Conversion_between_quaternions_and_Euler_angles'''
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


def normalize(angle: float) -> float:
    """ Returns angle (radians) between 0 and 2pi """
    while angle < 0.0:
        angle += 2. * math.pi
    while angle >= 2 * math.pi:
        angle -= 2. * math.pi
    return angle


def state_error(s_curr: np.array, s_ref: np.array) -> np.array:
    """ Return the error that reflects the proper angular distance """
    theta_err = 0
    if s_curr[2][0] - s_ref[2][0] > math.pi:
        theta_err = s_curr[2][0] - (s_ref[2][0] + 2 * math.pi)
    elif s_ref[2][0] - s_curr[2][0] > math.pi:
        theta_err = (s_curr[2][0] + 2 * math.pi) - s_ref[2][0]
    else:
        theta_err = s_curr[2][0] - s_ref[2][0]
    return np.array([[s_curr[0][0] - s_ref[0][0]], 
                     [s_curr[1][0] - s_ref[1][0]], 
                     [theta_err]])