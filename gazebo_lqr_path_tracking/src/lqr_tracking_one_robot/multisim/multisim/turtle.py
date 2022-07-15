#! /usr/bin/env python
import math
import os
from ament_index_python import get_package_share_directory
import rclpy, numpy
from re import M
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Pose, Point, Twist
from gazebo_msgs.srv import SpawnEntity
from nav_msgs.msg import Odometry

# Find path to 'calcs.py' so that it can be included
# Uses pathlib.Path() which finds the path of the current file
# .absolute() gives the absolute path
# .parent gives the directory of the file
import sys, pathlib
path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(path))
import calcs
from calcs import euler_from_quaternion

# the angular and linear margins allowed
global LINEAR_MARGIN
LINEAR_MARGIN = 0.2

# dt for the lqr and for the function calling the approach to it
global dt
dt = 0.2

global cur_wypt
cur_waypt = 0

class Turtle(Node):
    def __init__(self):
        super().__init__('turtle')
        # Callback groups
        self.multiple_clbk = ReentrantCallbackGroup()
        self.single_clbk = MutuallyExclusiveCallbackGroup()

        # Name of turtle
        self.name = self.get_name()

        # Startup parameters
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('pos_x', 3.0),
                                    ('pos_y', 0.0),
                                    ('pos_z', 0.0)
                                ]
                               )
        # set initial position
        self.pose = Pose()
    

        # Generate request to /spawn_entity service
        self.client = self.create_client(SpawnEntity, "/spawn_entity")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()
        self.send_request()
        
        # list of waypoints to follow as path
        # Initial waypoint will be the initial position of the robot
                         # values within the list will be the waypoints described as:
                         # [Point(x,y,z),[roll,pitch,yaw]]
        self.waypoints = []
        # Calculate waypoint path
        self.calc_waypoint_path()
        self.cur_waypt = 1

        self.approach_waypoint_timer = self.create_timer(dt, 
                                                        self.approach_waypoint,
                                                        callback_group=self.single_clbk)

        # Publisher to own cmd_vel
        self.pub_cmd_vel = self.create_publisher(Twist,
                                                 '/' + self.name + '/cmd_vel',
                                                 1)
    
        # Subscribe to own pose
        self.sub_pose = self.create_subscription(Odometry,
                                                 '/' + self.name + '/odom',
                                                 self.update_pose,
                                                 1)

        #self.get_logger().info('waypoints: '+str(self.waypoints))

    def send_request(self):
        """ Send request to /spawn_entity service """
        # Get path to the Turtlebot3 burger
        urdf_file_path = os.path.join(os.path.join(
                                        get_package_share_directory("turtlebot3_gazebo"), 
                                        "models",
                                        "turtlebot3_burger",
                                        "model.sdf"))

        self.req.name = self.get_name()
        self.req.xml = str(open(urdf_file_path, 'r').read())
        self.req.robot_namespace = self.get_name()
        self.req.reference_frame = "world"
        
        self.req.initial_pose = Pose()
        self.req.initial_pose.position.x = self.get_parameter('pos_x').get_parameter_value().double_value
        self.req.initial_pose.position.y = self.get_parameter('pos_y').get_parameter_value().double_value
        self.req.initial_pose.position.z = self.get_parameter('pos_z').get_parameter_value().double_value

        self.pose.position.x = self.req.initial_pose.position.x
        self.pose.position.y = self.req.initial_pose.position.y
        self.pose.position.z = self.req.initial_pose.position.z

        self.future = self.client.call_async(self.req)
        self.client.destroy()

        return self.future.result()
    def update_pose(self, new_odom: Odometry) -> Pose:
        """ Updates the Pose of the turtle """
        #self.get_logger().info(str(new_odom.pose.position))
        self.pose = new_odom.pose.pose

    def calc_waypoint_path(self):
        self.waypoints = calcs.discrete_path(calcs.r_circle, 0, 2*math.pi, 10, self.req.initial_pose)
        self.get_logger().info("Path: " + str(self.waypoints))
    
    def approach_waypoint(self):
        """ Use LQR control to approach the next waypoint """
        # define the needed variables for the lqr to operate according to the system
        # our robot's current point

        # Come full circle
        if self.cur_waypt >= len(self.waypoints) - 1:
            self.cur_waypt = 0

        actual_state_x = numpy.array([self.pose.position.x,
                                      self.pose.position.y,
                                      calcs.normalize(euler_from_quaternion(self.pose.orientation.x,
                                                                            self.pose.orientation.y,
                                                                            self.pose.orientation.z,
                                                                            self.pose.orientation.w)[2])])
        
        # R matrix - The control input cost matrix
        R = numpy.array([[0.001,   0],  # Penalty for linear velocity effort
                         [  0,   0.01]]) # Penalty for angular velocity effort
        # Q matrix - The state cost matrix.
        Q = numpy.array([[1.9, 0, 0],  # Penalize X position error 
                         [0, 1.9, 0],  # Penalize Y position error 
                         [0, 0, 0.7]]) # Penalize YAW ANGLE heading error
        # A and B matrixes - Expresses how the state of the system [x, y, yaw] changes 
        A = numpy.array([  [1.0,   0,   0],
                           [  0, 1.0,   0],
                           [  0,   0, 1.0]])  
        B = calcs.getB(actual_state_x[2], dt)

        # initiate the velocity vector
        u_star = numpy.array([0.0, 0.0])
        # get next waypoint
        dest = Point(x=self.waypoints[self.cur_waypt][0].x,
                     y=self.waypoints[self.cur_waypt][0].y,
                     z=self.waypoints[self.cur_waypt][0].z)
        yaw = calcs.normalize(self.waypoints[self.cur_waypt][1][2])

        # our robot's desired state
        desired_state_xf = numpy.array([dest.x, dest.y, yaw])

        # check if we're not close enough to the destination
        if calcs.dist(calcs.pose_to_pt(self.pose), dest) > LINEAR_MARGIN:
            u_star = calcs.lqr(actual_state_x, desired_state_xf, Q, R, A, B, dt)
            # calculate the velocity needed according to lqr
            cmd_vel = Twist()
            cmd_vel.linear.x = u_star[0]
            cmd_vel.angular.z = u_star[1]
            self.pub_cmd_vel.publish(cmd_vel)
        else:
            self.cur_waypt += 1
        # self.get_logger().info('current waypoint: '+str(dest))
        #self.get_logger().info('current position: '+str(self.pose))
        

def main(args = None):
    # Initiate rclpy interface session
    rclpy.init(args = args)
    # Instantiate pid_node
    turtle_node = Turtle()
    # Instantiate Executor
    executor = MultiThreadedExecutor()
    executor.add_node(turtle_node)
    # spin the node
    executor.spin()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
