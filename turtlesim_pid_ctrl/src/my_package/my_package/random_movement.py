import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose 
import random

class PID_control(Node):

	def __init__(self):
		# initiate your PID_control object
		super().__init__('pid')

		# create your publisher, which in this case will publish a velocity to the turtle
		self.publisher_vel = self.create_publisher(Twist, '/turtlesim1/turtle1/cmd_vel', 10)
		# inititate your position
		self.posX = 0
		self.posY = 0

		# create your subscriber, which in this case will subscribe to the position of the turtle
		self.subscriber_pos = self.create_subscription(Pose, '/turtlesim1/turtle1/pose', self.update_pose, 10)
		
		# create your time, that is when the function "get_pid" will be called.
		timer_period = 0.01 # timer will go off every 0.01 seconds
		self.timer = self.create_timer(timer_period, self.get_pid)	

	# this function will update the position of the turtle
	def update_pose(self,msg):
		self.posX = msg.x
		self.posY = msg.y

	# this function will eventually be the pid control function that will control the velocity of the turtle in order to get it to the wanted location
	def get_pid(self):
		
		# this will eventually be the velocity which will be controlled by the PID controller. Currently, in order to just check if we can control the movement of the turtle in simulation I've randumly assigned the velocity components every time the function is called
		msg = Twist()
		msg.linear.x = random.uniform(-10,10)
		msg.linear.y = random.uniform(-10,10)

		# publish the velocity
		self.publisher_vel.publish(msg)
	
		# this logger is intended on checking whether or not the position actually gets updated and could be used later for the PID control.
		self.get_logger().info('Subscribing X: "%s"' %str(self.posX))		
		self.get_logger().info('Subscribing Y: "%s"' %str(self.posY))

def main(args=None):
	# Initialize the rclpy library
	rclpy.init(args=args)

	# Create the node
	pid = PID_control()

	# Spin the node so the callback function is called.
	# Publish any pending messages to the topics.
	rclpy.spin(pid)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	pid.destroy_node()

	# Shutdown the ROS client library for Python
	rclpy.shutdown()


if __name__ == '__main__':
	main()
