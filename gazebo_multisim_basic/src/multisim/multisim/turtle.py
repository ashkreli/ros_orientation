#! /usr/bin/env python
import math, os, rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Pose, Quaternion, Point, Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity

# Find path to 'calcs.py' so that it can be included
# Uses pathlib.Path() which finds the path of the current file
# .absolute() gives the absolute path
# .parent gives the directory of the file
import sys, pathlib
from gazebo_multisim_interfaces.msg import TurtleName, TurtleNames
path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(path))
import calcs
from data_types import PoseTwist

global step_size
step_size = 0.1

global k_angular, k_linear
k_angular = 0.7
k_linear = 0.5

global ANGULAR_MARGIN, LINEAR_MARGIN
ANGULAR_MARGIN = 0.02
LINEAR_MARGIN = 0.1

# Physical constraints on robot
global MAX_LIN_VEL, MAX_ANG_VEL
MAX_LIN_VEL = 0.9
MAX_ANG_VEL = 2.5

class Turtle(Node):
    def __init__(self):
        super().__init__('turtle')
        # Callback groups
        self.multiple_clbk = ReentrantCallbackGroup()
        self.single_clbk = MutuallyExclusiveCallbackGroup()
        
        # Done with objective to reach reference?
        self.done = False

        # Name of turtle
        self.name = self.get_name()

        # Startup parameters
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('pos_x', 10.0),
                                    ('pos_y', 0.0),
                                    ('pos_z', 0.0),
                                    ('orient_x', 0.0),
                                    ('orient_y', 0.0),
                                    ('orient_z', 0.0),
                                    ('orient_w', 0.0),
                                    ('ref_x', 0.0),
                                    ('ref_y', 0.0),
                                    ('ref_z', 0.0),
                                ]
                               )
        # Generate request to /spawn_entity service
        self.client = self.create_client(SpawnEntity, "/spawn_entity")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()
        self.ref = Point()
        self.waypoint = Point()
        self.send_request()

        # Destroy client to service once service has been processed
        self.client.destroy()

        self.cmd_vel = Twist()
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
        
        # Subscribe to /ref topic for reference point
        self.ref_sub = self.create_subscription(Point,
                                                '/ref',
                                                self.update_ref,
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
        # **activated tells us whether or not PoseTwist is from a real message
        self.all_info = {}
    
    def pub_presence(self):
        """ Publish own name to '/new_turtle' topic """
        now_online = TurtleName()
        now_online.name = self.name
        self.pub_new_turtle.publish(now_online)
        # self.get_logger().info(self.name + " NOW ONLINE")

    def send_request(self):
        """ Send request to /spawn_entity service """
        # Get path to the Turtlebot3 burger description
        urdf_file_path = os.path.join(get_package_share_directory('multisim'),
                                     'config',
                                     'model.sdf'
                                     )

        self.req.name = self.get_name()
        self.req.xml = str(open(urdf_file_path, 'r').read())
        self.req.robot_namespace = self.get_name()
        self.req.reference_frame = "world"
        
        # Set parameters from the /config and fed in launch file
        self.req.initial_pose = Pose()
        self.req.initial_pose.position.x = self.get_parameter('pos_x').get_parameter_value().double_value
        self.req.initial_pose.position.y = self.get_parameter('pos_y').get_parameter_value().double_value
        self.req.initial_pose.position.z = self.get_parameter('pos_z').get_parameter_value().double_value
        
        self.req.initial_pose.orientation = Quaternion()
        self.req.initial_pose.orientation.x = self.get_parameter('orient_x').get_parameter_value().double_value
        self.req.initial_pose.orientation.y = self.get_parameter('orient_y').get_parameter_value().double_value
        self.req.initial_pose.orientation.z = self.get_parameter('orient_z').get_parameter_value().double_value
        self.req.initial_pose.orientation.w = self.get_parameter('orient_w').get_parameter_value().double_value

        # Let initial waypoint be where the robot currently is
        self.waypoint = calcs.pose_to_pt(self.req.initial_pose)

        self.future = self.client.call_async(self.req)

        # Now that request went through, instantiate all timers  
        self.pub_new_turtle_timer = self.create_timer(2.0, 
                                                      self.pub_presence,
                                                      callback_group=self.multiple_clbk)
        self.calc_waypoint_timer = self.create_timer(0.6, 
                                                     self.calc_waypoint,
                                                     callback_group=self.multiple_clbk)
        self.approach_waypoint_timer = self.create_timer(0.3, 
                                                         self.approach_waypoint,
                                                         callback_group=self.multiple_clbk)

        return self.future.result()
    def update_turtle_list(self, all_turtles_msg):
        """ Updates list of all turtle names as well as subscriptions """
        # self.get_logger().info("Current turtles: " + str(all_turtles_msg.turtle_names))
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

    def update_ref(self, new_ref):
        """ Updates the reference point if '/ref' topic is updated """
        if not calcs.pt_equal(self.ref, new_ref):
            self.done = False
        self.ref.x = new_ref.x
        self.ref.y = new_ref.y
        # self.get_logger().info("Reference updated: (" + str(new_ref.x) + ", " + str(new_ref.y) + ")")

    def generate_callback(self, name):
        """ 
            Generate a callback function to update a specific PoseTwist 
            given that a certain Odometry topic has been updated 
            (no other way to incorporate the name of the turtle being updated)
        """
        def update_pose_twist(new_odom: Odometry) -> None:
            """ Updates the PoseTwist of the turtle with name in field 'name' """
            if name not in self.all_info:
                self.all_info[name] = [PoseTwist(), False]
            self.all_info[name][1] = True
            self.all_info[name][0].pose = new_odom.pose.pose
            self.all_info[name][0].twist = new_odom.twist.twist
            # self.get_logger().info("PoseTwist updated")
        return update_pose_twist

    def calc_waypoint(self):
        """ 
            By gradient descent of superposed potential functions,
            finds the next waypoint for the robot, which will subsequently
            use PID or LQR to reach it  
        """
        if (self.name not in self.all_info) or (not (self.all_info[self.name])[1]) or self.done:
            return
        # Position of turtle currently
        cur_pt = calcs.pose_to_pt((self.all_info[self.name])[0].pose)
        # Add repulsive potential gradients from other turtles
        rep_grad = Point()
        for turtle in self.all_names:
            if not (turtle == self.name):
                rep_grad = calcs.add_vecs(rep_grad, 
                               calcs.repulsive_potential_grad(
                                  cur_pt,
                                  calcs.pose_to_pt(self.all_info[turtle][0].pose),
                                  self.all_info[self.name][0]))
        # self.get_logger().info("Repulsion: (" + str(rep_grad.x) + ", " + str(rep_grad.y) + ')')
        ref_grad = calcs.reference_potential_grad(self.ref, cur_pt)
        tot_grad = calcs.add_vecs(ref_grad, rep_grad)
        candidate_wypt = Point(x=cur_pt.x + step_size * -(tot_grad.x),
                               y=cur_pt.y + step_size * -(tot_grad.y),
                               z=cur_pt.z + step_size * -(tot_grad.z)
                            )
        # self.get_logger().info(self.name + ": (" + str(candidate_wypt.x) + ", " + str(candidate_wypt.y) + ')')
        if calcs.dist(candidate_wypt, self.ref) < LINEAR_MARGIN:
            self.waypoint = self.ref
        else:
            self.waypoint = candidate_wypt
    
    def approach_waypoint(self):
        """ Use PID control to approach the current waypoint """
        # Return if no valid position information yet
        if self.name not in self.all_info or not (self.all_info[self.name])[1]:
            return
        # Position of turtle currently
        cur_pose = (self.all_info[self.name])[0].pose
        cmd_vel = Twist()
        # Stop if reasonably close to reference
        if calcs.dist(calcs.pose_to_pt(cur_pose), self.ref) < LINEAR_MARGIN and not self.done:
            cmd_vel.angular.z = 0.0
            cmd_vel.linear.x = 0.0
            self.done = True
            return
        # Rotate and move forward towards waypoint (or transitory reference)
        theta_ref = math.atan2(self.waypoint.y-cur_pose.position.y, 
                               self.waypoint.x-cur_pose.position.x)
        theta_ref = calcs.normalize(theta_ref)
        # Convert Quaternion into Euler angles, and retrieve the angle
        # about the z-axis, since that corresponds to rotation of vehicle
        theta_cur = calcs.euler_from_quaternion(cur_pose.orientation.x,
                                                cur_pose.orientation.y,
                                                cur_pose.orientation.z,
                                                cur_pose.orientation.w)[2]
        theta_cur = calcs.normalize(theta_cur)
        ang_err = theta_ref - theta_cur
        # self.get_logger().info("Theta cur: " + str(theta_cur))
        # self.get_logger().info("Theta ref: " + str(theta_ref))
        if abs(ang_err) > ANGULAR_MARGIN:
            if abs(ang_err) > math.pi:
                if theta_cur < theta_ref:
                    theta_cur += 2 * math.pi
                else:
                    theta_ref += 2 * math.pi
            if abs(theta_ref - theta_cur) < (2. * math.pi - abs(theta_ref - theta_cur)):
                cmd_vel.angular.z = k_angular * (theta_ref - theta_cur)
            else:
                cmd_vel.angular.z = k_angular * (theta_cur - theta_ref)
            # cmd_vel.angular.z = k_angular * (theta_cur - theta_ref)
        else:
            cmd_vel.angular.z = 0.0
        
        # Add repulsive potential gradients from other turtles
        rep_grad = Point()
        for turtle in self.all_names:
            if not (turtle == self.name):
                rep_grad = calcs.add_vecs(rep_grad, 
                               calcs.repulsive_potential_grad(
                                  calcs.pose_to_pt(self.all_info[self.name][0].pose),
                                  calcs.pose_to_pt(self.all_info[turtle][0].pose)))
        # Recover repulsion force magnitude
        rep_mag = calcs.dist(rep_grad, Point())
        # Linear vel inversely proportional to repulsion, modulated by distance from reference
        if rep_mag > 0.0:
            cmd_vel.linear.x = (1 / rep_mag) * calcs.dist(calcs.pose_to_pt(cur_pose), self.ref)
        else:
            cmd_vel.linear.x = k_linear * calcs.dist(calcs.pose_to_pt(cur_pose), self.ref)
        # Cap to maximum velocities
        if cmd_vel.linear.x > MAX_LIN_VEL:
            cmd_vel.linear.x = MAX_LIN_VEL
        if cmd_vel.angular.z > MAX_ANG_VEL:
            cmd_vel.angular.z = MAX_ANG_VEL
        # Publish desired velocity
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