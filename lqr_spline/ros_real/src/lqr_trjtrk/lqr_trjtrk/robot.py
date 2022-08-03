import numpy as np
import os, rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Pose, Quaternion, Point, Twist, PoseStamped

# Find path to 'calcs.py' so that it can be included
# Uses pathlib.Path() which finds the path of the current file
# .absolute() gives the absolute path
# .parent gives the directory of the file
import sys, pathlib
path = pathlib.Path(__file__).absolute().parent.parent.parent.parent.parent
path = os.path.join(path, 'utils')
sys.path.append(str(path))
from calcs import euler_from_quaternion
import calcs, lqr, sim_move, spline_int, trajectory_shapes
from spline_int import attach_t, gen_s_u

global MAX_LIN, MAX_ANG
MAX_LIN = 0.22
MAX_ANG = 2.0

global I, N
N = 30
I = 3

f = open('states.txt', 'w')

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
                                                1)
        
        # Subscribe to own pose_twist
        self.sub_pose = self.create_subscription(PoseStamped,
                                                '/vrpn_client_node/' + self.name + '/pose',
                                                self.update_pose,
                                                1)

        # list of all turtlebot states init at start point
        self.states = []

        self.u_refs = []
        self.s_refs = []

        self.traj_path()

        self.get_logger().info("dt: " + str(self.dt))

        # Callback to the to movement towards waypoint
        self.approach_waypoint_timer = self.create_timer(round(self.dt / I, 1),
                                                         self.approach_waypoint,
                                                         callback_group=self.multiple_clbk)
        self.next_waypoint_timer = self.create_timer(self.dt,
                                                     self.next_waypoint,
                                                     callback_group=self.multiple_clbk)
    def next_waypoint(self):
        """ Pops the most recent waypoint to give priority to the future ones """
        if len(self.s_refs) > 1 and len(self.u_refs) > 1: 
            self.s_refs.pop(0)
            self.u_refs.pop(0)

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
            f.write(np.array2string(new_state) + '\n')
            #f.write(str(new_state[0][0]) + '\n' + str(new_state[1][0]) + '\n' + str(new_state[2][0]))
            self.states.append(new_state)
            # Pull first control input given by LQR
            u = lqr.lqr_traj_track_dare(self.states, self.s_refs, self.u_refs, self.dt / I)[0]
            # u = lqr.lqr_evol_ref_cvxpy(self.states, self.s_refs, self.dt / I)[0]
        cmd_vel = Twist()
        cmd_vel.linear.x = u[0][0]
        cmd_vel.angular.z = u[1][0]
        self.pub_cmd_vel.publish(cmd_vel)


    def update_pose(self, msg: PoseStamped):
        self.pose = msg.pose
    
    def traj_path(self):
        """ Pick one of the trajectories by commenting out the rest """
        # Circle trajectory informed by the physical constraints of the Turtlebot
        '''s_refs, u_refs, dt = trajectory_shapes.circle_inf(center=(1.5, -1.5), 
                                                          radius=1.2, 
                                                          max_lin=MAX_LIN/2, 
                                                          max_ang=MAX_ANG, 
                                                          num_steps=N)'''
        # Line trajectory
        '''s_refs, u_refs = trajectory_shapes.straight_line((0, 0), (2.0, -3.0), 25)
        dt = 1'''
        # Spline-interpolated trajectory through some arbitrary points
        waypts = [np.array([[0], [0]]), 
                  np.array([[1], [-1]]), 
                  np.array([[2], [-2]]), 
                  np.array([[3], [-3]])]
        max_vels = [MAX_LIN*0.6, MAX_LIN*0.6, MAX_LIN*0.6]
        waypts = attach_t(waypts, max_vels)
        s_refs, u_refs = gen_s_u(waypts, N)
        dt = (waypts[-1][0] - waypts[0][0]) / N
        #self.get_logger().info("dt: " + str(dt))
        self.s_refs = s_refs
        #self.get_logger().info(str(s_refs))
        self.u_refs = u_refs
        #self.get_logger().info(str(u_refs))
        self.dt = round(dt, 1)
        # Write trajectory points to 'refs.txt'
        with open('refs.txt', 'w') as f1:
            for s_ref in self.s_refs:
                f1.write(np.array2string(s_ref) + '\n')

    


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

