#! /usr/bin/env python
'''
    Code anticipates that robots will only come online, never offline
    Trying to add this measure is not very enlightening for the time
    being since we do not have a real network, nor a namespacing schema
    that we would like to use

    A Turtles node holds the list of all the turtle names in the system
    This is to simulate a more dynamic real life scenario of when we bring
    up turtles, to register them in a central place so that the topics related
    to their speed, position are accessible without externally assuming their
    names(though their subsequent attribute topics will have the same names
    which we can assume)
'''
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Point
import sys, pathlib
path = pathlib.Path(__file__).parent.absolute()
sys.path.append(str(path))
from gazebo_multisim_interfaces.msg import TurtleNames, TurtleName

timer_period = 0.5  # seconds

class Turtles(Node):
    def __init__(self):
        super().__init__('turtles_node')
        # define callback group to handle multiple threads
        self.multiple_clbk = ReentrantCallbackGroup()
        self.single_clbk = MutuallyExclusiveCallbackGroup()
        # turtles that are online
        self.all_turtles = []

        # subscribe to /new_turtle topic which has the latest turtle
        # that is incoming (may be backed up initially)
        self.inc_turtle_sub = self.create_subscription(TurtleName, 
                                                       '/new_turtle', 
                                                       self.sub_new_turtle, 
                                                       1)
        
        # create publisher to /all_turtles which holds a list of all 
        self.inc_turtle_pub = self.create_publisher(TurtleNames, 
                                                    '/all_turtles',
                                                    1)
        self.timer_inc_turtle_pub = self.create_timer(1.0, 
                                                      self.pub_new_turtle,
                                                      callback_group=self.multiple_clbk)
        
        # create publisher to /ref topic holding reference to be reached
        self.ref_pub = self.create_publisher(Point,
                                            '/ref',
                                            1)
        self.ref = Point()
        self.timer_ref_pub = self.create_timer(0.5, 
                                               self.pub_ref, 
                                               callback_group=self.multiple_clbk)

        # set initial reference from parameters
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('ref_x', 0.0),
                                    ('ref_y', 0.0),
                                    ('ref_z', 0.0)
                                ])
        self.ref.x = self.get_parameter('ref_x').get_parameter_value().double_value
        self.ref.y = self.get_parameter('ref_y').get_parameter_value().double_value
        self.ref.z = self.get_parameter('ref_z').get_parameter_value().double_value
        
        self.ref_pub.publish(self.ref)

    def sub_new_turtle(self, new_turtle_name):
        """ Update the list of all turtles which are in the system 
            within the node """
        if new_turtle_name.name not in self.all_turtles:
            # self.get_logger().info("Adding " + new_turtle_name.name + " to /all_turtles")
            self.all_turtles.append(new_turtle_name.name)
    
    def pub_new_turtle(self):
        """ Update the topic that holds the list of all turtle names """
        msg = TurtleNames()
        msg.turtle_names = self.all_turtles
        # self.get_logger().info("PUBLISHING ALL TURTLES: " + str(self.all_turtles))
        self.inc_turtle_pub.publish(msg)
    
    def pub_ref(self):
        """ Publish the reference point to /ref topic """
        # self.get_logger().info("Publishing ref: (" + str(self.ref.x) + ", " + str(self.ref.y) + ")")
        self.ref_pub.publish(self.ref)

def main(args = None):
    # Initiate rclpy interface session
    rclpy.init(args = args)
    # Instantiate pid_node
    turtles_node = Turtles()
    executor = MultiThreadedExecutor()
    executor.add_node(turtles_node)
    # 'spin' means to, through rclpy interface, communicate via DDS protocol
    # with other nodes until the process is done
    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtles_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()