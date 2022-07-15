turtle.py: Source code for Turtlebot node, of which many can be spawned
turtles.py: source for node that holds the names of all turtles online (support for only one)
calcs.py: File with helper functions for calculations made to determine next waypoint and approaching it
data_types.py: Holds custom data type PoseTwist, which encapsulates the pose and the twist of a turtle

## Turtle Node
Subscriber to `/all_turtles`
    Calls `update_turtle_list()`
Publisher to `/new_turtle`
    Calls pub_presence
Client to `/spawn_entity`
Subscriber to `/odom` of itself and all other turtles
    New subscriptions are added when update_turtle_list is called
    Each subscription needs its own callback function to update
    the information that the Turtle currently possesses about 
Publisher to `/cmd_vel` of itself
Subscriber to `/ref`

The assumption is that a Turtle does not initially know how many/what other
turtles will be present in the simulation, and must be able to dynamically subscribe
to the topics associated with their pose. Each node makes its own calculations based
on the information it has. Therefore the system is semi-distributed, because there is
still a central server that posts each turtle's positions, rather than having each turtle
sense what other turtles are near it.

Subscribing to the poses of other turtles involves calling a callback function that
updates the information it has on other turtles upon receiving incoming messages.
ROS assumes that the callback functions only have one input: the message itself.
However, it is a problem not being able to know what specific turtle's information
has been updated, since the message comes from a Gazebo subprocess that cannot be
modified to include in the message the name of the turtle. Therefore, we need to
dynamically generate the callback function for each subscription that modifies the
specific turtle, hence the `generate_callback()` function.

Next, we calculate the next waypoint based on where the robot currently is, and
make it so that the waypoint is getting closer to the reference. As explained in
the `README.md` file, the system that any given robot experiences is a superposition
of potential functions of the relevant elements that "affect" it. The reference point
is the minimum of the potential function (insert). We choose this function since the
potential droops down from all directions to the reference.

The robot in question also experiences a potential function from all other robots.
From any robot in particular, the potential function is the following:

Note that it sharply peaks at the location of the other robot, as a way of repelling
the robot since it is doing gradient descent.

Doing gradient descent requires taking the gradient of all the superposed potential
functions, which is simply adding the gradient of each individual potential and 
adding them. The gradient of the repulsive potential defined between two robots can 
be found in `calcs.py`, and so is the gradient of the attractive potential defined 
between the reference and the current position of the robot. The addition of all the 
gradients (and their subsequent negation for gradient descent) is done in the 
`calc_waypoint()` function in `turtle.py`. A new waypoint is calculated at a fixed
and frequent rate, specified in the timer defined in `__init__()`

Approaching the waypoint requires setting an appropriate `cmd_vel`, which has a linear
component and an angular component. Robots in this simulation only move forward, despite
in real life being able to move backwards as well (conscious design choice). They also
rotate either clockwise or counterclockwise. The linear velocity was made to be inversely
proportional to the magnitude of the repulsion force, but also modulated by the distance
from the reference point. In the case where there is only one robot (or zero repulsion),
the robot will have a speed proportional to the distance from the reference point.


## Turtles Node
Subscriber to /new_turtle
Publisher to /all_turtles
Publisher to /ref