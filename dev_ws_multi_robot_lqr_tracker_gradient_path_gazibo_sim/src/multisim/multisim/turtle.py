#! /usr/bin/env python
import math
import os, rclpy, numpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Pose, Quaternion, Point, Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_multisim_interfaces.msg import TurtleName, TurtleNames

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
dt = 0.5

class Turtle(Node):
    def __init__(self):
        super().__init__('turtle')
        # cCllback groups
        self.multiple_clbk = ReentrantCallbackGroup()
        self.single_clbk = MutuallyExclusiveCallbackGroup()
        # Done with objective to reach reference?
        self.done = False

        # Name of turtle
        self.name = self.get_name()

        # Startup parameters
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('pos1_x', -5.0),
                                    ('pos1_y', 0.0),
                                    ('pos1_z', 0.0),
                                    ('pos2_x', 15.0),
                                    ('pos2_y', 15.0),
                                    ('pos2_z', 0.0),
                                    ('pos3_x', 7.0),
                                    ('pos3_y', 3.0),
                                    ('pos3_z', 0.0),
                                    ('pos4_x', 7.0),
                                    ('pos4_y', 0.0),
                                    ('pos4_z', 0.0),
                                    ('orient_x', 0.0),
                                    ('orient_y', 0.0),
                                    ('orient_z', 0.0),
                                    ('orient_w', 0.0),
                                    ('ref_x', 10.0),
                                    ('ref_y', 5.0),
                                    ('ref_z', 0.0),
                                ]
                               )
        # get the refrence point of the robots
        self.ref = Point()
        self.ref.x = self.get_parameter('ref_x').get_parameter_value().double_value
        self.ref.y = self.get_parameter('ref_y').get_parameter_value().double_value
        self.ref.z = self.get_parameter('ref_z').get_parameter_value().double_value

        # Generate request to /spawn_entity service
        self.client = self.create_client(SpawnEntity, "/spawn_entity")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()
        self.send_request()

        # Publisher to own cmd_vel
        self.pub_cmd_vel = self.create_publisher(Twist,
                                                '/' + self.name + '/cmd_vel',
                                                10)
        # List of all turtles' names
        self.all_names = []
        
        # Subscribe to /all_turtles topic to update list
        self.all_turtles_sub = self.create_subscription(TurtleNames,
                                                        '/all_turtles',
                                                        self.update_turtle_list,
                                                        1)
        
        
        # Publish to /new_turtle topic to signal its presence
        self.pub_new_turtle = self.create_publisher(TurtleName,
                                                    '/new_turtle',
                                                    1)

        # Dict of subscriptions to all turtles' /odom topics
        # Format: {turtle_name (str): subscription (Subscription)}
        self.subs = {}

        # PoseTwists of all turtles
        # Format: {turtle_name (str): [PoseTwist, activated** (bool)]}
        # **activated tells us whether the PoseTwist is generic or from
        # a real message
        self.all_info = {}
    
    def pub_presence(self):
        """ Publish own name to '/new_turtle' topic """
        now_online = TurtleName()
        now_online.name = self.name
        self.pub_new_turtle.publish(now_online)

    def set_params(self):
        " Set parameters from the /config and fed in launch file "
        self.req.initial_pose = Pose()
        self.req.initial_pose.position.x = self.get_parameter('pos'+self.name[-1]+'_x').get_parameter_value().double_value
        self.req.initial_pose.position.y = self.get_parameter('pos'+self.name[-1]+'_y').get_parameter_value().double_value
        self.req.initial_pose.position.z = self.get_parameter('pos'+self.name[-1]+'_z').get_parameter_value().double_value
        
        self.req.initial_pose.orientation = Quaternion()
        self.req.initial_pose.orientation.x = self.get_parameter('orient_x').get_parameter_value().double_value
        self.req.initial_pose.orientation.y = self.get_parameter('orient_y').get_parameter_value().double_value
        self.req.initial_pose.orientation.z = self.get_parameter('orient_z').get_parameter_value().double_value
        self.req.initial_pose.orientation.w = self.get_parameter('orient_w').get_parameter_value().double_value

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
        self.set_params()

        self.future = self.client.call_async(self.req)
        self.client.destroy()
        # Now that request went through, instantiate all timers  
        self.pub_new_turtle_timer = self.create_timer(3.0, 
                                                      self.pub_presence,
                                                      callback_group=self.multiple_clbk)
        # lists of waypoints to follow as path
        # Initial waypoint will be the initial position of the robot
                         # values within the list will be the waypoints described as:
                         # [Point(x,y,z),[roll,pitch,yaw]]
        self.waypoints = [[calcs.pose_to_pt(self.req.initial_pose),[0.0,0.0,0.0]]]
        
        # Calculate waypoint path
        self.calc_waypoint_path()
        
        # approach the waypoints created
        self.approach_waypoint_timer = self.create_timer(dt, 
                                                         self.approach_waypoint,
                                                         callback_group=self.single_clbk)

        return self.future.result()

    def update_turtle_list(self, all_turtles_msg):
        """ Updates list of all turtle names as well as subscriptions """
        # Update list of all names
        self.all_names = all_turtles_msg.turtle_names
        # Update corresponding subscriptions
        # Subscribe to own Odometry because it is given by external node
        for turtle in self.all_names:
            if turtle not in self.subs:
                func = self.generate_callback(turtle)
                self.all_info[turtle] = [PoseTwist(), False]
                self.subs[turtle] = self.create_subscription(Odometry,
                                                            '/' + turtle + '/odom',
                                                            func,
                                                            1)

    def generate_callback(self, name):
        """ Generate a callback function to update a specific PoseTwist 
            given that a certain Odometry topic has been updated 
            - no other way to incorporate the name of the turtle being 
            updated """
        def update_pose_twist(new_odom: Odometry) -> None:
            """ Updates the PoseTwist of the turtle with name in field 'name' """
            if name not in self.all_info:
                self.all_info[name] = [PoseTwist(), False]
            (self.all_info[name])[1] = True
            (self.all_info[name])[0].pose = new_odom.pose.pose
            (self.all_info[name])[0].twist = new_odom.twist.twist
        return update_pose_twist

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
            V_x = numpy.array([numpy.absolute(V[0]),0.0,0.0])
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
        return
    
    def approach_waypoint(self):
        """ Use LQR control to approach the next waypoint """
        if self.name not in self.all_info or not (self.all_info[self.name])[1]:
            return
        # Position of turtle currently
        cur_pose = (self.all_info[self.name])[0].pose
        
        # redefine the linear margine
        lin_marge = LINEAR_MARGIN
        # define the needed variables for the lqr to opporate acording to the system
        actual_state_x = numpy.array([cur_pose.position.x,cur_pose.position.y,
                                                         calcs.normalize(euler_from_quaternion([cur_pose.orientation.x,
                                                         cur_pose.orientation.y,
                                                         cur_pose.orientation.z,
                                                          cur_pose.orientation.w],
                                                         'rxyz')[2])])
        
        # R matrix - The control input cost matrix
        R = numpy.array([[0.01,   0],  # Penalty for linear velocity effort
                    [  0, 0.05]]) # Penalty for angular velocity effort
        # Q matrix - The state cost matrix.
        Q = numpy.array([[1.2, 0, 0],  # Penalize X position error 
                                    [0, 1.4, 0],  # Penalize Y position error 
                                    [0, 0.5, 0]]) # Penalize YAW ANGLE heading error  
        # A and B matrixes - Expresses how the state of the system [x,y,yaw] changes 
        A = numpy.array([  [1.0,  0,   0],
                                        [  0, 1.0,   0],
                                        [  0,  0, 1.0]])  
        B = calcs.getB(actual_state_x[2], dt)

        # make sure they don't crash
        rep_grad = Point()
        for turtle in self.all_names:
            # Add repulsive potential gradients from other turtles
            if not (turtle == self.name):
                calcs.add_vecs(rep_grad, 
                              calcs.repulsive_potential_grad(
                                calcs.pose_to_pt((self.all_info[turtle])[0].pose), 
                                                 calcs.pose_to_pt(cur_pose)))
            if (calcs.dist(calcs.pose_to_pt((self.all_info[turtle])[0].pose), self.ref) <= lin_marge):
                lin_marge *= 2

        # initiate the velocity vector
        u_star = numpy.array([0.0,0.0])
        # get next waypoint
        dest = Point()
        dest.x = self.waypoints[0][0].x - step_size*rep_grad.x
        dest.y = self.waypoints[0][0].y - step_size*rep_grad.y
        dest.z = self.waypoints[0][0].z - step_size*rep_grad.z
        yaw = self.waypoints[0][1][2]

        # our robot's desired state
        desired_state_xf = numpy.array([dest.x,dest.y,yaw])

        # check if we're not close enough to the destination
        if calcs.dist(calcs.pose_to_pt(cur_pose), self.ref) > lin_marge:
            u_star = calcs.lqr(actual_state_x, desired_state_xf, Q, R, A, B, dt)
            
        # if the list of waypoints only has the last point
        if(len(self.waypoints) > 1):
            # get rid of already calculated point
            self.waypoints.pop(0)
        
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
