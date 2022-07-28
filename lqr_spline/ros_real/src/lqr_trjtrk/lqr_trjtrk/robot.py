import math
import os, rclpy, numpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Pose, Quaternion, Point, Twist, PoseWithCovariance, TwistWithCovariance, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity

# Find path to 'calcs.py' so that it can be included
# Uses pathlib.Path() which finds the path of the current file
# .absolute() gives the absolute path
# .parent gives the directory of the file
import sys, pathlib
path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(path))

class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        # Callback groups
        self.multiple_clbk = ReentrantCallbackGroup()
        self.single_clbk = MutuallyExclusiveCallbackGroup()

        self.name = self.get_name()

        # set initial position
        self.pose = Pose()

        # Publisher to own cmd_vel
        self.pub_cmd_vel = self.create_publisher(Twist,
                                                '/' + self.name + '/cmd_vel',
                                                10)
        
        # Subscribe to own pose_twist
        self.sub_pose = self.create_subscription(PoseStamped,
                                                '/vrpn_client_node' + '/mobile_sensor_' + self.name[-1] + '/pose',
                                                self.update_pose,
                                                10)
    
    def update_pose(self, msg: PoseStamped):
        self.pose = msg.pose

    


def main(args = None):
    # Initiate rclpy interface session
    rclpy.init(args = args)
    # Instantiate pid_node
    robot_node = Robot()
    # Instantiate Executor
    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)
    # spin the node
    executor.spin()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

