# Concepts

## Launch file



## Config file



## Executors and Callback Groups

Operating systems need to manage what order processes run in, as well as whether they
could/should run in parallel or in series. In ROS2, we have the abstraction mechanism
of Executors and Callback Groups which we must make use of in the case of a Turtle
node, which has several processes that need to be able to run in parallel.
