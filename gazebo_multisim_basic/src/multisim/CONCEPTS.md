# Concepts

## Launch Files

Launching multiple ROS 2 nodes with a launch file to reduce the number of terminals 
one needs to open

## Configuration Files

The `src/config` directory holds the description of the Turtlebot in simulation,
and also a parameter configuration file that specifies desired initial positions
of the Turtlebots to be simulated.

## Executors and Callback Groups

Operating systems need to manage what order processes run in, as well as whether they
could/should run in parallel or in series. In ROS2, we have the abstraction mechanism
of Executors and Callback Groups which we must make use of in the case of a Turtle
node, which has several processes that need to be able to run in parallel.
