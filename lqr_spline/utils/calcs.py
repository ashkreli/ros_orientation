import math
from typing import Tuple

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

def normalize(theta: float) -> float:
    """ Return angle expressed b/t 0 and 2pi """
    if 0.0 <= theta < 2 * math.pi:
        return theta
    elif theta >= 2 * math.pi:
        return theta - math.floor(theta / 2 / math.pi) * 2 * math.pi
    elif theta < 0.0:
        return -theta + math.floor(-theta / 2 / math.pi) * 2 * math.pi

