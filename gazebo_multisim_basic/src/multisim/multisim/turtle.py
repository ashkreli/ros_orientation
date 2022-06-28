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

# Find path to 'calcs.py' so that it can be included
# Uses pathlib.Path() which finds the path of the current file
# .absolute() gives the absolute path
# .parent gives the directory of the file
import sys, pathlib
# sys.path.append("~/ros_orientation1/gazebo_multisim/install/gazebo_multisim_interfaces")
from gazebo_multisim_interfaces.msg import TurtleName, TurtleNames
path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(path))
import calcs
from data_types import PoseTwist

global step_size
step_size = 0.2

global k_angular, k_linear
k_angular = 0.6
k_linear = 0.3

global ANGULAR_MARGIN, LINEAR_MARGIN
ANGULAR_MARGIN = 0.02
LINEAR_MARGIN = 0.2

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
        # self.get_logger().info("Name: "+ self.name)

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
        self.send_request()

        # Publisher to own cmd_vel
        self.pub_cmd_vel = self.create_publisher(Twist,
                                                '/' + self.name + '/cmd_vel',
                                                10)
        
        # Publish to /new_turtle topic to signal its presence
        self.pub_new_turtle = self.create_publisher(TurtleName,
                                                    '/new_turtle',
                                                    1)
        
        self.pub_new_turtle_timer = self.create_timer(3.0, 
                                                      self.pub_presence,
                                                      callback_group=self.multiple_clbk)

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

        # Initial waypoint will be the reference
        self.waypoint = self.ref
        
        self.calc_waypoint_timer = self.create_timer(0.5, 
                                                     self.calc_waypoint,
                                                     callback_group=self.single_clbk)
        self.approach_waypoint_timer = self.create_timer(0.5, 
                                                         self.approach_waypoint,
                                                         callback_group=self.single_clbk)

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
        # self.get_logger().info(self.name + " NOW ONLINE")

    def set_params(self):
        " Set parameters from the /config and fed in launch file "
        self.req.initial_pose = Pose()
        self.req.initial_pose.position.x = self.get_parameter('pos_x').get_parameter_value().double_value
        self.req.initial_pose.position.y = self.get_parameter('pos_y').get_parameter_value().double_value
        self.req.initial_pose.position.z = self.get_parameter('pos_z').get_parameter_value().double_value
        
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
        self.ref.x = new_ref.x
        self.ref.y = new_ref.y
        # self.get_logger().info("Reference updated: (" + str(new_ref.x) + ", " + str(new_ref.y) + ")")

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
            # self.get_logger().info("PoseTwist updated")
        return update_pose_twist

    def calc_waypoint(self):
        """ 
            By gradient descent of superposed potential functions,
            finds the next waypoint for the robot, which will subsequently
            use PID or LQR to reach it  
        """
        if (self.name not in self.all_info) or (not (self.all_info[self.name])[1]):
            return
        # Position of turtle currently
        cur_pt = calcs.pose_to_pt((self.all_info[self.name])[0].pose)
        # Add repulsive potential gradients from other turtles
        rep_grad = Point()
        for turtle in self.all_names:
            if not (turtle == self.name):
                calcs.add_vecs(rep_grad, 
                              calcs.repulsive_potential_grad(
                                calcs.pose_to_pt((self.all_info[turtle])[0].pose), 
                                                 cur_pt))
        ref_grad = calcs.reference_potential_grad(self.ref, cur_pt)
        self.waypoint.x = cur_pt.x + step_size * (-rep_grad.x + -ref_grad.x)
        self.waypoint.y = cur_pt.y + step_size * (-rep_grad.y + -ref_grad.y)
        self.waypoint.z = cur_pt.z + step_size * (-rep_grad.z + -ref_grad.z)
        # self.get_logger().info("Waypoint: (" + str(self.waypoint.x) + ", " + str(self.waypoint.y) + ')')
        return
    
    def approach_waypoint(self):
        """ Use PID control to approach the current waypoint """
        if self.name not in self.all_info or not (self.all_info[self.name])[1]:
            # self.get_logger().info("Returned prematurely")
            return
        # Position of turtle currently
        cur_pose = (self.all_info[self.name])[0].pose
        cmd_vel = Twist()
        # self.get_logger().info("Distance: " + str(calcs.dist(calcs.pose_to_pt(cur_pose), self.ref)))
        # Stop if reasonably close to reference
        if calcs.dist(calcs.pose_to_pt(cur_pose), self.ref) < 0.2 and not self.done:
            cmd_vel.angular.z = 0.0
            cmd_vel.linear.x = 0.0
            self.done = True
            # self.get_logger().info("Done!")
        # Rotate and move forward towards waypoint (or transitory reference)
        elif calcs.dist(calcs.pose_to_pt(cur_pose), 
                        self.ref) > LINEAR_MARGIN and not self.done: 
            dist_err1 = calcs.dist(calcs.pose_to_pt(cur_pose), self.waypoint)
            dist_err2 = calcs.dist(calcs.pose_to_pt(cur_pose), self.ref)
            theta_ref = numpy.arctan2(self.waypoint.y-cur_pose.position.y, 
                                      self.waypoint.x-cur_pose.position.x)
            # Convert Quaternion into Euler angles, and retrieve the angle
            # about the z-axis, since that corresponds to rotation of vehicle
            theta_cur = euler_from_quaternion([cur_pose.orientation.x,
                                               cur_pose.orientation.y,
                                               cur_pose.orientation.z,
                                               cur_pose.orientation.w],
                                              'rxyz')[2]
            # theta_cur = calcs.normalize(theta_cur)
            ang_err = theta_ref - theta_cur
            # self.get_logger().info("Theta cur: " + str(theta_cur))
            # self.get_logger().info("Theta ref: " + str(theta_ref))
            if abs(ang_err) > ANGULAR_MARGIN:
                if abs(ang_err) < abs(2 * math.pi - ang_err):
                    cmd_vel.angular.z = k_angular * ang_err
                else:
                    cmd_vel.angular.z = k_angular * (2 * math.pi - ang_err)
            else:
                cmd_vel.angular.z = 0.0
            cmd_vel.linear.x = k_linear * dist_err2
        self.pub_cmd_vel.publish(cmd_vel)
        # self.get_logger().info("Approaching waypoint")

        
        

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