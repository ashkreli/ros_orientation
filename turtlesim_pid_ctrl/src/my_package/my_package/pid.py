import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose 
import os
import sys


class PID_control(Node):

	def __init__(self,destX,destY,kp_const,ki_const,kd_const):
		# initiate your PID_control object
		super().__init__('pid')

		# create your publisher, which in this case will publish a velocity to the turtle
		self.publisher_vel = self.create_publisher(Twist, '/turtlesim1/turtle1/cmd_vel', 10)
		
		# get the goal coordinates
		self.goal_x, self.goal_y,self.kp,self.ki,self.kd = destX,destY,kp_const,ki_const,kd_const


		# inititate your position
		self.posX = 0
		self.posY = 0

		# initiate your distances
		self.distance_x = []
		self.distance_x.append(self.goal_x - self.posX)
		self.distance_y = []
		self.distance_y.append(self.goal_y - self.posY)
		self.distanceX = self.distance_x[0]
		self.distanceY = self.distance_y[0]
		self.total_distanceX = 0.0
		self.total_distanceY = 0.0
		self.diff_distanceX = self.distance_x[0]
		self.diff_distanceY = self.distance_y[0]

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
		
		# calculate the distances
		self.distance_x.append(self.goal_x - self.posX)
		self.distance_y.append(self.goal_y - self.posY)
		self.distanceX = self.distance_x[-1]
		self.distanceY = self.distance_y[-1]
		self.total_distanceX += self.distance_x[-1]
		self.total_distanceY += self.distance_y[-1]
		self.diff_distanceX = self.distance_x[-1]-self.distance_x[-2]
		self.diff_distanceY = self.distance_y[-1]-self.distance_y[-2]

		# set the velocity according to the PID calculation
		msg = Twist()
		msg.linear.x = self.kp*self.distanceX + self.ki*self.total_distanceX + self.kd*self.diff_distanceX
		msg.linear.y = self.kp*self.distanceY + self.ki*self.total_distanceY + self.kd*self.diff_distanceY

		# this will print out the position so I can see if it actually went to the correct spot
		self.get_logger().info('Subscribing X: "%s"' %str(self.posX))		
		self.get_logger().info('Subscribing Y: "%s"' %str(self.posY))

		# publish the velocity
		self.publisher_vel.publish(msg)
	
	def getkey(self):
		goal_x = float(destX)
		goal_y = float(destX)
		kp = float(kp_const)
		ki = float(ki_const)
		kd = float(kd_const)
		return goal_x,goal_y,kp,ki,kd

def main(args=None):
	# Initialize the rclpy library
	rclpy.init(args=args)

	# open text file containing relavent user inputs
	text_file = open("/home/eliannesacher/lab_stuff/dev_ws_old/src/my_package/my_package/my_file.txt", "r")
	# load string into file
	f = text_file.read()
	# close the file
	text_file.close()
	
	# Initialize the destination location (according to the initializing values in the text file)
	destX = float(f.split('destX')[1].split('$')[1])
	destY = float(f.split('destY')[1].split('$')[1])

	# Initializing the kp, ki, and kd values (according to the initializing values in the text file)
	kp_const = float(f.split('kp_const')[1].split('$')[1])
	ki_const = float(f.split('ki_const')[1].split('$')[1])
	kd_const = float(f.split('kd_const')[1].split('$')[1])
	
	
	
	# Create the node
	pid = PID_control(destX,destY,kp_const,ki_const,kd_const)

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
