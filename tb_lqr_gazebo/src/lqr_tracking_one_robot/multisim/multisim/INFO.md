turtle.py: Source code for Turtlebot node
calcs.py: File with helper functions for calculations made to determine next waypoint and approaching it
data_types.py: Holds custom data type PoseTwist, which encapsulates the pose and the twist of a turtle

## Turtle Node
Subscriber to /odom of itself
    New subscriptions are added when update_turtle_list is called
    Each subscription needs its own callback function to update
    the information that the Turtle currently possesses about 
Publisher to /cmd_vel of itself

Subscribing to pose of the turtle involves calling a callback function that 
updates the information it has on its current location upon receiving incoming messages. 
ROS assumes that the callback functions only have one input: the message itself. 

Next, we calculate the waypoints path based on where the robot initially is, and 
make it so that the waypoints are getting closer to the reference. As explained in 
the README.md file, the system that any the robot experiences is 
of a potential functions of the relevant elements that "affect" it (the refrence point and the current point). Note, that the reference point
is selected as the minimum of the potential function. We choose this function since the
potential droops down from all directions to the reference.

Doing gradient descent requires taking the gradient of all the superposed potential
functions, which is simply adding the gradient of each individual potential, since
the gradient is linear. The gradient of the attractive potential defined between the 
reference and the current position of the robot can be found in calcs.py. In calc_waypoints_path,
the new waypoints are calculated (the number of which is fixed by the user).

Approaching the waypoint was done by LQR, which includes
the rotation of the robot so that it moves forward in the right direction towards
the waypoints. Every time the robot moves to the next waypoint until it is in a close enough proximity to 
the final waypoint. Orientation in Gazebo is given in Quaternions, which is not the most
intuitive way to do so, so the tf_transformations library was used to convert a
pose orientation given in Quaternions into Euler angles.
