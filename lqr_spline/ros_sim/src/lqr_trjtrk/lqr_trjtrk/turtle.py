import os, rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Pose, Quaternion, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity

# Uses pathlib.Path() which finds the path of the current file
# .absolute() gives the absolute path
# .parent gives the directory of the file
import sys, pathlib, os
path = pathlib.Path(__file__).absolute().parent.parent.parent.parent.parent
print(str(path))
path = os.path.join(path, 'utils')
sys.path.append(str(path))
from calcs import euler_from_quaternion
import calcs, lqr, sim_move, spline_int, trajectory_shapes
from spline_int import attach_t, gen_s_u

# Number of times between waypoints that LQR controller will be used
global I
# Spline
# I = 2
# Line / Circle
I = 1

# Number of waypoints making up a trajectory
global N
# Spline
# N = 50
# Line / Circle
N = 100
# Velocity Constraints
global MAX_LIN, MAX_ANG
MAX_LIN = 0.5
MAX_ANG = 1.7

f = open('states.txt', 'w')
f_actual_inputs = open('inputs_actual.txt', 'w')

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
                                    ('pos_x', 8.0),
                                    ('pos_y', 0.0),
                                    ('pos_z', 0.0),
                                    ('orient_x', 0.0),
                                    ('orient_y', 0.0),
                                    ('orient_z', 0.0),
                                    ('orient_w', 0.0)]
                               )
        # set initial position
        self.pose = Pose()
        self.pose.position.x = self.get_parameter('pos_x').get_parameter_value().double_value
        self.pose.position.y = self.get_parameter('pos_y').get_parameter_value().double_value
        self.pose.position.z = self.get_parameter('pos_z').get_parameter_value().double_value
        self.pose.orientation = Quaternion()
        self.pose.orientation.x = self.get_parameter('orient_x').get_parameter_value().double_value
        self.pose.orientation.y = self.get_parameter('orient_y').get_parameter_value().double_value
        self.pose.orientation.z = self.get_parameter('orient_z').get_parameter_value().double_value
        self.pose.orientation.w = self.get_parameter('orient_w').get_parameter_value().double_value
        
        # Generate request to /spawn_entity service
        self.client = self.create_client(SpawnEntity, "/spawn_entity")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()
        self.send_request()
        
        # Publisher to own cmd_vel
        self.pub_cmd_vel = self.create_publisher(Twist,
                                                '/' + self.name + '/cmd_vel',
                                                1)
   
        # Subscribe to own pose_twist
        self.sub_pose = self.create_subscription(Odometry,
                                                '/' + self.name + '/odom',
                                                self.update_pose_twist,
                                                1)
       
        # list of all turtlebot states init at start point
        self.states = []

        self.u_refs = []
        self.s_refs = []
        self.dt = 0
        # Calculate trajectory path
        self.traj_path()
        # Callback to the to movement towards waypoint
        self.approach_waypoint_timer = self.create_timer(self.dt / I,
                                                         self.approach_waypoint,
                                                         callback_group=self.multiple_clbk)
        self.next_waypoint_timer = self.create_timer(self.dt,
                                                     self.next_waypoint,
                                                     callback_group=self.multiple_clbk)
        self.record_state_timer = self.create_timer(0.1,
                                              self.record_state,
                                              callback_group=self.multiple_clbk)
    def set_params(self):
        " Set parameters from /config and fed via launch file "
        self.req.initial_pose = Pose()
        self.req.initial_pose.position.x = self.get_parameter('pos_x').get_parameter_value().double_value
        self.req.initial_pose.position.y = self.get_parameter('pos_y').get_parameter_value().double_value
        self.req.initial_pose.position.z = self.get_parameter('pos_z').get_parameter_value().double_value
       
    def send_request(self):
        """ Send request to /spawn_entity service """
        # Get path to the Turtlebot3 burger description
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
    
    def update_pose_twist(self, new_odom: Odometry) -> None:
        """ Updates the PoseTwist of the turtle """
        self.pose = new_odom.pose.pose
        self.twist = new_odom.twist.twist

    def traj_path(self):
        """ Pick one of the trajectories by commenting out the rest """
        # Circle trajectory informed by the physical constraints of the Turtlebot
        s_refs, u_refs, dt = trajectory_shapes.circle_inf(center=(0.0, 0.0), 
                                                          radius=6.0, 
                                                          max_lin=MAX_LIN, 
                                                          max_ang=MAX_ANG, 
                                                          num_steps=N)
        # Spline-interpolated trajectory through some arbitrary points
        '''waypts = [np.array([[1], [1]]), 
                  np.array([[2], [4]]), 
                  np.array([[5], [2]]), 
                  np.array([[6], [6]])]
        max_vels = [MAX_LIN/4, MAX_LIN/4, MAX_LIN/4]
        waypts = attach_t(waypts, max_vels)
        s_refs, u_refs = gen_s_u(waypts, N)
        dt = (waypts[-1][0] - waypts[0][0]) / N'''
        # Linear Trajectory
        '''waypts = [np.array([[1], [1]]), 
                  np.array([[5], [6]])]
        max_vels = [MAX_LIN/1.5, MAX_LIN/1.5]
        waypts = attach_t(waypts, max_vels)
        s_refs, u_refs = gen_s_u(waypts, N)
        dt = (waypts[-1][0] - waypts[0][0]) / N'''
        
        self.s_refs = s_refs
        #self.get_logger().info(str(s_refs))
        self.u_refs = u_refs
        #self.get_logger().info(str(u_refs))
        self.dt = dt

        # Write trajectory points to 'refs.txt'
        with open('refs.txt', 'w') as f1:
            for s_ref in s_refs:
                f1.write(np.array2string(s_ref) + '\n')
        # Write trajectory inputs to 'inputs_ideal.txt'
        with open('inputs_ideal.txt', 'w') as f2:
            for u_ref in u_refs:
                f2.write(np.array2string(u_ref) + '\n')
    
    def next_waypoint(self):
        """ Pops the most recent waypoint to give priority to the future ones """
        if len(self.s_refs) > 1 and len(self.u_refs) > 1: 
            self.s_refs.pop(0)
            self.u_refs.pop(0)

    def record_state(self):
        """ Record current state for plots later """
        state = np.array([[self.pose.position.x],
                                  [self.pose.position.y],
                                  [calcs.normalize(
                                    euler_from_quaternion(
                                        self.pose.orientation.x,
                                        self.pose.orientation.y,
                                        self.pose.orientation.z,
                                        self.pose.orientation.w)[2])]])
        f.write(np.array2string(state) + '\n')

    def approach_waypoint(self):
        """ Use LQR controller to approach the ideal trajectory """
        # Define the needed variables for LQR to operate based on system state
        # Default command velocity vector
        u = np.array([[0.0], [0.0]])
        # check if there are still references to follow
        if len(self.s_refs) > 1 and len(self.u_refs) > 1:     
            # Append current state to state history
            new_state = np.array([[self.pose.position.x],
                                  [self.pose.position.y],
                                  [calcs.normalize(
                                    euler_from_quaternion(
                                        self.pose.orientation.x,
                                        self.pose.orientation.y,
                                        self.pose.orientation.z,
                                        self.pose.orientation.w)[2])]])
            self.states.append(new_state)
            # Pull first control input given by LQR
            u = lqr.lqr_traj_track_dare(self.states, self.s_refs, self.u_refs, self.dt / I)[0]
            f_actual_inputs.write(np.array2string(u) + '\n')
            # u = lqr.lqr_evol_ref_cvxpy(self.states, self.s_refs, self.dt / I)[0]
        cmd_vel = Twist()
        cmd_vel.linear.x = u[0][0]
        cmd_vel.angular.z = u[1][0]
        self.pub_cmd_vel.publish(cmd_vel)
       
def main(args = None):
    # Initiate rclpy interface session
    rclpy.init(args = args)
    # Instantiate turtle_node
    turtle_node = Turtle()
    # Instantiate Executor
    executor = MultiThreadedExecutor()
    executor.add_node(turtle_node)
    # Spin the node
    executor.spin()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_node.destroy_node()
    rclpy.shutdown()
    # Close the data files to be plotted later
    f.close()
    f_actual_inputs.close()

if __name__ == '__main__':
    main()
