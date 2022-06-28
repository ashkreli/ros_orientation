## gazebo_multisim_interfaces

It is best practice to include the custom interfaces used in a project in a separate package.
In our case, we have interface definitions for message TurtleName, which is posted by a
TurtleBot to the '/new_turtle' topic, to which the 'turtles' node is subscribed.

Subsequently, the 'turtles' node reads the possibly new turtle that came in and updates the
list of turtles that are in the simulation. This is done by posting to the '/all_turtles'
topic an updated list of all the turtles' names (strings). This list is how all the turtles
know who else is in the network and as a result set up subscriptions to the topics relating
to their Poses, since the topic names follow a format of '/turtle_name/attribute_name'.

The idea is to have all the robots think independently while having access to all the locations 
and velocities of other robots (decentralized approach as opposed to centralized).