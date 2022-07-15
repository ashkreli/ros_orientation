#! /usr/bin/env python
import math
import os, rclpy, numpy
from re import M
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Pose, Quaternion, Point, Twist, PoseWithCovariance, TwistWithCovariance, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# Find path to 'calcs.py' so that it can be included
# Uses pathlib.Path() which finds the path of the current file
# .absolute() gives the absolute path
# .parent gives the directory of the file
import sys, pathlib
path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(path))
import calcs
from data_types import PoseTwist

# the step size that we will use for calculation
global step_size
step_size = 0.4

# the number of waypoint
global num_of_waypoints
num_of_waypoints = 50 # arbitrary number

# the angular and linear margins allowed
global LINEAR_MARGIN
LINEAR_MARGIN = 0.5

# dt for the lqr and for the function calling the approach to it
global dt
dt = 0.2

class Turtle(Node):
    def __init__(self):
        super().__init__('turtle')
        # Callback groups
        self.multiple_clbk = ReentrantCallbackGroup()
        self.single_clbk = MutuallyExclusiveCallbackGroup()

        # Name of turtle
        self.name = self.get_name()

        self.get_logger().info(self.name)

        # Startup parameters
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('ref_x', 3.0),
                                    ('ref_y', -3.0),
                                    ('ref_z', 0.0),
                                    ('orient_x', 0.0),
                                    ('orient_y', 0.0),
                                    ('orient_z', 0.0),
                                    ('orient_w', 0.0),
                                ]
                               )

        # set initial position
        self.pose = Pose()

        # Publisher to own cmd_vel
        self.pub_cmd_vel = self.create_publisher(Twist,
                                                '/' + self.name + '/cmd_vel',
                                                10)
    
        # get refrence point from file
        self.ref = Point()
        self.ref.x = self.get_parameter('ref_x').get_parameter_value().double_value
        self.ref.y = self.get_parameter('ref_y').get_parameter_value().double_value
        self.ref.z = self.get_parameter('ref_z').get_parameter_value().double_value
    
        # Subscribe to own pose_twist
        self.sub_pose = self.create_subscription(PoseStamped,
                                                            '/vrpn_client_node' + '/mobile_sensor_' + self.name[-1] + '/pose',
                                                            self.update_pose,
                                                            10)
        
        # lists of waypoints to follow as path
        # Initial waypoint will be the initial position of the robot
                         # values within the list will be the waypoints described as:
                         # [Point(x,y,z),[roll,pitch,yaw]]
        self.waypoints = [[calcs.pose_to_pt(self.pose),[0.0,0.0,0.0]]]

        # Calculate waypoint path
        self.calc_waypoint_path()

        #self.get_logger().info('waypoints: '+str(self.waypoints))

        # Callback to the to movement towards waypoint*
        self.approach_waypoint_timer = self.create_timer(dt, 
                                                         self.approach_waypoint,
                                                         callback_group=self.single_clbk)

    def update_pose(self, new_odom):
        """ Updates the PoseTwist of the turtle """
        #self.get_logger().info(str(new_odom.pose.position))
        self.pose = new_odom.pose

    def calc_waypoint_path(self):
        """
            By gradient descend of superposed potential function,
            finds all the waypoints for the robot to get to the destination
        """
        # instansiate the points used later on
        next_waypoint = Point()

        # get all #num_of_waypoints for the path
        for i in range(0,num_of_waypoints):

            # calculate the refrence gradient function
            ref_grad = calcs.reference_potential_grad(self.ref, self.waypoints[-1][0])
            # find the next linear coordinates of the next waypoint based on the gradient function
            next_waypoint.x = self.waypoints[-1][0].x - step_size*ref_grad.x
            next_waypoint.y = self.waypoints[-1][0].y - step_size*ref_grad.y
            next_waypoint.z = self.waypoints[-1][0].z - step_size*ref_grad.z

            # vector of the wanted direction to next point
            V = numpy.array([next_waypoint.x-self.waypoints[-1][0].x,next_waypoint.y-self.waypoints[-1][0].y,next_waypoint.z-self.waypoints[-1][0].z])

            # projection vectors
            V_proj_xy = numpy.array([V[0],V[1],0.0])
            V_x = numpy.array([10.0,0.0,0.0])
            # calculating the yaw angle and the pitch angle (basic multi-var)
            pitch_arccos = numpy.dot(V/numpy.linalg.norm(V),V_proj_xy/numpy.linalg.norm(V_proj_xy))
            yaw_arccos = numpy.dot(V_proj_xy/numpy.linalg.norm(V_proj_xy),V_x/numpy.linalg.norm(V_x))
            # set the roll of the next point to zero
            next_point_roll = 0.0
            # check that it's within bounderies and set
            if (pitch_arccos < 1.0) and (pitch_arccos > -1.0):
                next_point_pitch = float(numpy.arccos(pitch_arccos))
            else:
                next_point_pitch = self.waypoints[-1][1][1]
            if (yaw_arccos < 1.0) and (yaw_arccos > -1.0):
                next_point_yaw = float(numpy.arccos(yaw_arccos))
            else:
                next_point_yaw = self.waypoints[-1][1][2]
            # set the orientation of the next point
            # append the next waypoint
            self.waypoints.append([Point(x = next_waypoint.x, y = next_waypoint.y, z = next_waypoint.z),[next_point_roll,next_point_pitch,next_point_yaw]])
        self.waypoints.append([self.ref,[self.waypoints[-1][1][0],self.waypoints[-1][1][1],self.waypoints[-1][1][2]]])

    
    def approach_waypoint(self):
        """ Use LQR control to approach the next waypoint """
        # define the needed variables for the lqr to opporate acording to the system
        # our robot's current point

        actual_state_x = numpy.array([self.pose.position.x,self.pose.position.y,
                                                         calcs.normalize(euler_from_quaternion([self.pose.orientation.x,
                                                         self.pose.orientation.y,
                                                         self.pose.orientation.z,
                                                          self.pose.orientation.w],
                                                         'rxyz')[2])])
        
        # R matrix - The control input cost matrix
        R = numpy.array([[0.001,   0],  # Penalty for linear velocity effort
                    [  0, 0.1]]) # Penalty for angular velocity effort
        # Q matrix - The state cost matrix.
        Q = numpy.array([[1.7, 0, 0],  # Penalize X position error 
                                    [0, 1.7, 0],  # Penalize Y position error 
                                    [0, 0.6, 0]]) # Penalize YAW ANGLE heading error  
        # A and B matrixes - Expresses how the state of the system [x,y,yaw] changes 
        A = numpy.array([  [1.0,  0,   0],
                                        [  0, 1.0,   0],
                                        [  0,  0, 1.0]])  
        B = calcs.getB(actual_state_x[2], dt)

        # initiate the velocity vector
        u_star = numpy.array([0.0,0.0])
        # get next waypoint
        dest = Point()
        dest.x = self.waypoints[0][0].x
        dest.y = self.waypoints[0][0].y
        dest.z = self.waypoints[0][0].z
        yaw = calcs.normalize(self.waypoints[0][1][2])

        # our robot's desired state
        desired_state_xf = numpy.array([dest.x,dest.y,yaw])

        # check if we're not close enough to the destination
        if calcs.dist(calcs.pose_to_pt(self.pose), self.ref) > LINEAR_MARGIN:
            u_star = calcs.lqr(actual_state_x, desired_state_xf, Q, R, A, B, dt)
        # if the list of waypoints only has the last point
        if(len(self.waypoints) > 1):
            # get rid of already calculated point
            self.waypoints.pop(0)
        
        #self.get_logger().info('current waypoint: '+str(dest))
        #self.get_logger().info('current position: '+str(self.pose))

        # calculate the velocity needed according to lqr
        cmd_vel = Twist()
        cmd_vel.linear.x = u_star[0]
        cmd_vel.angular.z = u_star[1]
        self.pub_cmd_vel.publish(cmd_vel)
        

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
