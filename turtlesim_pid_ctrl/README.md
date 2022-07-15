# **HOW TO:** Creating PID Controlled Location Seeker Simulation with ROS2 and Turtlebot
###**Step-by-step explanation**

Note: this assumes that from the published ROS2 tutorials all of the beginner and intermediate tutorials were complete.

## 1. Configure the environment and create a workspace:

Note: for a deeper explanation of this look through the following tutorials:

<https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html>

<https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>

<https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>

<https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>

1. Open terminal (Ctrl+T)
2. Paste the following with inputting your_domian_id (a number between 0-100):

`echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc`

`echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc`

3. Install colcon by pasting the following in the terminal:
4. `sudo apt install python3-colcon-common-extensions`
5. Create, clone, and build an example workspace by pasting the following in the terminal:

`mkdir -p ~/dev_ws/src`

`cd ~/dev_ws/src`

`git clone https://github.com/ros/ros_tutorials.git -b foxy-devel`

`# cd if you're still in the ``src`` directory with the ``ros_tutorials`` clone`

`cd ..`

`rosdep install -i --from-path src --rosdistro foxy -y`

`colcon build`

6. Create a package by pasting the following in the terminal:

`cd ~/dev_ws/src`

`ros2 pkg create --build-type ament_python --node-name my_node my_package`

`cd ~/dev_ws`

`colcon build`

## 2. Understanding (relevant) nodes and topics/services/parameters/actions:

Note: for a deeper explanation of this look through the following tutorial:

<https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#rqt-graph>

To understand the relevant layout of the project, we must understand which nodes we must instantiate and which topics/services/parameters/actions we’ll need to use. Since all we need is one simulation window and one Turtlebot and all we want to achieve is the movement of the said Turtlebot in the simulation window we must investigate these relationships through rqt.

1. Open two more terminals:

   * In the first one, paste the following: `ros2 run turtlesim turtlesim_node`

   * In the second, paste the following: `ros2 run turtlesim turtle_teleop_key`

2. Move the turtle through the second terminal by using the arrows.

3. Paste the following in the terminal to see all the relevant topics in a none graphical view: `ros2 topic list -t`

4. This indicates that you’ll need to be using the /turtle1/pose and /turtle1/cmd_vel topics in your code. To understand exactly which components you will use within these toptics, paste the following commands in the termina (replace ‘topic type’ with the relevant topic type):

`ros2 interface show <topic type>`

For example:

`ros2 interface show turtlesim/msg/Pose`

## 3. Create a basic launch file consisting of the relevant nodes:

Note: for a deeper explanation of this look through the following tutorial:

<https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html>

1. Open a terminal from your package (if you followed the steps above that should be “my_package”), and create a launch folder where you will create and write your launch file. You may paste the following into the terminal:

`mkdir launch`

`touch launch/turtlesim_pid.launch.py`

Note: You could name your launch file whatever you want, but you should have the file ending with ‘.launch.py’ since that is consistent with standard practice.

2. The first thing you must have in your launch file is the common imports. I automatically paste the following at the beginning of every launch file (Note: you might need to add some):

`import os`

`from ament_index_python import get_package_share_directory`

`from launch import LaunchDescription`

`from launch.actions import DeclareLaunchArgument`

`from launch.actions import IncludeLaunchDescription`

`from launch.actions import GroupAction`

`from launch.launch_description_sources import PythonLaunchDescriptionSource`

`from launch.substitutions import LaunchConfiguration`

`from launch.substitutions import TextSubstitution`

`from launch_ros.actions import Node`

`from launch_ros.actions import PushRosNamespace`

3. Before you write your `generate_launch_description` function, you need to understand exactly how many and what nodes you will need. In this case, we will have one node that will be the window (turtlesim) and one node that will handle the turtle’s movement. For the former, you will have the “usual” code that you’ve been using throughout the tutorials:

`Node(`

`            package='turtlesim',`

`            namespace='turtlesim1',`

`            executable='turtlesim_node',`

`            name = 'sim'`

`        )`

For the latter, you will have to reassign the relevant package, relevant namespace, and relevant executable. In my case, the package name is `“my_package”` and the code I wrote to generate the movement is named `“pid.py”` meaning that the second node’s description will be as follows:

`Node(`

`            package='my_package',`

`            namespace='pid',`

`            executable='pid'`

`        )`

4. Putting the steps 2 and 3 together, the complete code within the launch file for this case could be the following:

`import os`

`from ament_index_python import get_package_share_directory`

`from launch import LaunchDescription`

`from launch.actions import DeclareLaunchArgument`

`from launch.actions import IncludeLaunchDescription`

`from launch.actions import GroupAction`

`from launch.launch_description_sources import PythonLaunchDescriptionSource`

`from launch.substitutions import LaunchConfiguration`

`from launch.substitutions import TextSubstitution`

`from launch_ros.actions import Node`

`from launch_ros.actions import PushRosNamespace`

`def generate_launch_description():`

`    return LaunchDescription([`

`        Node(`

`            package='turtlesim',`

`            namespace='turtlesim1',`

`            executable='turtlesim_node',`

`            name = 'sim'`

`        ),`

`        Node(`

`            package='my_package',`

`            namespace='pid',`

`            executable='pid'`

`        )`

`    ])`

You could go ahead and paste this into your launch file, but note that the names should be consistent with the ones you’ve selected.

5. After creating your launch file, you need to add the file to the `data_files` section of your `setup.py`  file. Paste the following into the `setup.py` file instead of what is currently in the “data_files” line (which should be line 9):

`data_files=[`

`        ('share/ament_index/resource_index/packages',`

`            ['resource/' + package_name]),`

`        ('share/' + package_name, ['package.xml']),`

`        (os.path.join('share', package_name, 'launch'),`

`         glob(os.path.join('launch', '\*.launch.py')))`

Note: if you haven’t followed the convention of calling your launch file with a name that ends with “`.launch.py`” you will have to change the `os.path.join` command to fit your file name.

6. You must also change your entry points to contain the scripts you’ve used. In my case, I only wrote one additional code file; therefore I only need to paste the following into the `setup.py` file instead of what is currently in the “'console_scripts'” line (which should be line 22):

`'console_scripts': [`

`            'pid = multisim.pid:main'`

`        ],`

7. Like always (as you’ve done plenty of times throughout the tutorial), within the `setup.py` file, change everywhere it says TODO to the relevant information (your name, your email, proper description, and proper license {“Apache License 2.0”}.
8. You will likely always have a dependency on `ros2launch` and `rclpy`. You must add these dependencies to your `package.xml` file. In this case, since you will be using `turtlesim` and `geometry_msgs` (which you’ve seen is necessary through step “2. Understanding (relevant) nodes and topics/services/parameters/actions”), you must add them as well. To some these two up, you must add the following to your `package.xml` file:

`  <exec_depend>rclpy</exec_depend>`

`  <exec_depend>geometry_msgs</exec_depend>`

`  <exec_depend>turtlesim</exec_depend>`

`  <exec_depend>ros2launch</exec_depend>`

9. Like always (as you’ve done plenty of times throughout the tutorial), within the `package.xml` file, change everywhere it says TODO to the relevant information (your name, your email, proper description, and proper license {“Apache License 2.0”}.

## 4. Write the actual code (pid.py):

Note: for a deeper explanation of this look through the following tutorial:

<https://automaticaddison.com/how-to-write-a-ros2-publisher-and-subscriber-python-foxy/>

1. Open a terminal from the script folder of your package (if you followed the previous steps it will be under “`dev_ws/src/my_package/my_package`”) and re-name “`my_code.py`” to “`pid.py`” or whatever you called it in your launch file.
2. The general build of the script file should consist of all imports that you need for your code (`os, numpy, random, etc.`) and the ROS2 specific ones. You will likely always have is “`import rclpy`” and “`from rcply.node import Node`” in addition to the relevant topic/services/parameters/action specific ones. In this case, since we will be subscribing and publishing the position and the velocity, we also need to import “`from geometry_msgs.msg import Twist`” and “`from turtlesim.msg import Pose`”. 

How do you know that these are the additional import messages necessary?

As you recall from the “2. Understanding (relevant) nodes and topics/services/parameters/actions” section, every topic has a certain topic type which lets us know where we can import the relevant libraries.

How do you ‘translate’ the topic type to the import statement?

For the example, after we pasted the following command in the terminal in the “2. Understanding (relevant) nodes and topics/services/parameters/actions” section: “`ros2 topic list -t`”, we got a list of active topics. Of which, we understood that the following two topics will be relevant to our code:

`/turtlesim1/turtle1/cmd_vel [geometry_msgs/msg/Twist]`

`/turtlesim1/turtle1/pose [turtlesim/msg/Pose]`

Breaking this down to its components, the `Twist` function can be imported from `geometry_msgs.msg`  and the `Pose` function can be imported from `turtlesim.msg`.

This gives us the following connection between a topic type and its import:

If topic type = `[a/b/c]` then its import statement will be = `from a.b import c`.

My full import portion ended up looking as follows:

`import rclpy` 

`from rclpy.node import Node`

`from geometry_msgs.msg import Twist` 

`from turtlesim.msg import Pose` 

`import random`

3. After writing the import section of the code, you move to write the portion of your class. The class will always receive “`Node`” as input.
4. Before you start writing the code for the class, think about what you need to subscribe to and what you need to publish. In this case, since we will be controlling the velocity of the turtle and we’ll evaluate the velocity based on the position of the turtle, we should be publishing the velocity and subscribing to the position. Now, knowing this, we can move to write the `__init__` function of the class, which must account for these subscriptions/publications.
5. As with every class, the class will consist of an `__init__` function and other helper functions. Per convention, =the `__init__` function will receive “`self`” as input. The init function will consist of the following:

   a. First, you will initiate your PID control object by using the following code: `super().__init__(‘pid’)`.
   b. Then you need to create a publisher to publish the velocity of the turtle. To do so, recall the relevant function name (“`Twist`”) and the relevant topic (“`/turtlesim1/turtle1/cmd_vel`”). Connecting these will give you the following code line:

`self.publisher_vel = self.create_publisher(Twist, '/turtlesim1/turtle1/cmd_vel', 10)`

6. Then, you move to creating the subscriber to subscribe to the position of the turtle. For good practice, you should instantiate the position to a default one. In this case, I set it to zero:

`self.posX = 0`

`self.posY = 0`

Then, you write the code for the subscription. Recall the function name and the relevant topic. Since you want to update this subscription every time it is changed, you must create a function to do so. This means, that the `create_sbscription` function will also have the input of that function (in this case I called it “`update_pose`”. This will give the following line of code:

`self.subscriber_pos = self.create_subscription(Pose, '/turtlesim1/turtle1/pose', self.update_pose, 10)`

7. Now, since you will want the function to be called every specific portion of the time, you need to create a timer that calls the callback function:

`timer_period = 0.01 # timer will go off every 0.01 seconds`

`self.timer = self.create_timer(timer_period, self.get_pid)`	

The full `__init__` function’s code will look like this:

`def __init__(self):`

`	super().__init__('pid')`

`	self.publisher_vel = self.create_publisher(Twist, '/turtlesim1/turtle1/cmd_vel', 10)`

`	self.posX = 0`

`	self.posY = 0`

`	self.subscriber_pos = self.create_subscription(Pose, '/turtlesim1/turtle1/pose', self.update_pose, 10)`
\`


`	timer_period = 0.01 # timer will go off every 0.01 seconds`

`	self.timer = self.create_timer(timer_period, self.get_pid)`	

8. As mentioned before, the `init` function calls a function that updates the position subscribed. That function will receive `self` and also `msg`. `msg` will be provided through the subscription and will contain the components of `Pose`. In step 6 of the “2. Understanding (relevant) nodes and topics/services/parameters/actions” section, you found the different components of `Pose`. The relevant ones for our purposes were `x` and `y`, which are of float type. This means, that in order to update the component we’ve created (`posX` and `posY`) you can just equate them to the `x` and `y` values of `msg`. The code for this function ends up looking as follows:

`def update_pose(self,msg):`

`self.posX = msg.x`

`self.posY = msg.y`

9. Before you actually write your PID control function’s code, you should first just try to randomly move the turtle on the simulation screen. This function will be the callback function mentioned before and will consist of the following:

   a. Creating the `Twist` type `msg` that will have the necessary velocity components (which are, according to step 6 of the “2. Understanding (relevant) nodes and topics/services/parameters/actions” section findings, `“linear.x`” and “`linear.y`”. You should set these fields to some float values (in my case I used the `random.uniform` function to generate this value). This portion of the code will look as follows:

`msg = Twist()`

`msg.linear.x = random.uniform(-10,10)`

`msg.linear.y = random.uniform(-10,10)`

 b. Then, you need to publish this `msg`:

`self.publisher_vel.publish(msg)`

10. This should cover the publishing part. Since we are still not doing the PID control and just randomly assigning the velocity component, we don’t use the subscription that we’ve instantiated. Nonetheless, since we are interested in knowing whether or not we’ve successfully created this subscription to be used later on, we can use the `get_logger()` function to print out the subscribed information in the terminal and see if it actually works. We can do so by adding the following code to the callback function:

`self.get_logger().info('Subscribing X: "%s"' %str(self.posX))`

`self.get_logger().info('Subscribing Y: "%s"' %str(self.posY))`

The complete code of the callback function should look something like this:

`def get_pid(self):`		

`	msg = Twist()`

`	msg.linear.x = random.uniform(-10,10)`

`	msg.linear.y = random.uniform(-10,10)`

`	self.publisher_vel.publish(msg)`
\`


`	self.get_logger().info('Subscribing X: "%s"' %str(self.posX))`		

`	self.get_logger().info('Subscribing Y: "%s"' %str(self.posY))`

11. Steps 3 through 7 should have concluded the class creation portion of the code. As always. We should define a main function and call the said main function. Since this is pretty self explanatory, I won’t go into the details, but the code should look something like this:

`def main(args=None):`

`	# Initialize the rclpy library`

`	rclpy.init(args=args)`

`	# Create the node`

`	pid = PID_control()`

`	# Spin the node so the callback function is called.`

`	# Publish any pending messages to the topics.`

`	rclpy.spin(pid)`

`	# Destroy the node explicitly`

`	# (optional - otherwise it will be done automatically`

`	# when the garbage collector destroys the node object)`

`	pid.destroy_node()`

`	# Shutdown the ROS client library for Python`

`	rclpy.shutdown()`

`if __name__ == '__main__':`

`	main()`

12. After following all the previous steps, you should be able to build your package, source your code, and launch it by pasting the following in your workspace terminal:

`colcon build`

`. install/setup.bash`

`ros2 launch my_package turtlesim_pid.launch.py`

This should run the simulation. The turtle should be randomly moving and the terminal should have the outputs of its current X and Y coordinates.

13. Now, you can move to edit the callback function so it will move towards the PID control. Since this should be handled on your own without additional ROS2-specific requirements, I won’t go through this step by step, but the code will be included in the relevant git repository.

