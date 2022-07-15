""" 
    Define some data types to make the calcs cleaner in the 'turtle.py'
    code as well as the one in 'calcs.py' 
"""

from geometry_msgs.msg import Pose, Twist


class PoseTwist():
    """ Encapsulate Pose and Twist information in one structure """
    def __init__(self):
        self._pose = Pose()
        self._twist = Twist()
    @property
    def pose(self):
        return self._pose
    @pose.setter
    def pose(self, pose):
        self._pose = pose
    @property
    def twist(self):
        return self._twist
    @twist.setter
    def twist(self, twist):
        self._twist = twist