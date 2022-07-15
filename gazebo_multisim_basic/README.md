## gazebo_multisim_basic
#### Code and docs by Arba Shkreli 
#### Email: ashkreli@college.harvard.edu
#### GitHub: ashkreli

This project is meant to demonstrate various important concepts in ROS2 that can be made use of heavily in simulation 
and in real robotics systems.
In a completed project, we have how to:
- Launch multiple nodes with a Launch file
- Use `.yaml` files to configure nodes with parameters
- Make use of Callback groups to handle processes that run in parallel and in series (i.e. managing concurrency)
- Call a service
- Subscribe/publish to topics
- Dynamically have each robot understand what other robots are in the system (so that the topics corresponding to 
their Pose and cmd_vel can be accessed)

Goal: Multiple robots (in our case TurtleBot3 'burger' models), given the same reference position to reach, 
must arrive and remain in the neighborhood of the reference point with reasonable trajectories and in the 
meantime not collide. 

An approach we took was to assign 2D potential functions to all objects in the system, where the system is taken to be 
all the robots (that know each other's positions under a common coordinate system) and the reference point. The reference 
point has an associated potential function with a minimum at the reference itself, while each robot has one that opposes 
it at the location of the robot. That is, they repel other robots. Each robot computes gradient descent on the superposition 
of the reference potential function as well as the potential functions of the other robots.

The platform that this code was built on is an Ubuntu 20.04 VM on Windows 11.  
Requirements:
- ROS 2 Version 'Foxy' (https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- Gazebo (Follow instructions on https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)
- Python 3.8

More in-depth walkthrough of the code is included in the relevant subdirectories for convenience.

Things that need to be adjusted in future commits:
- Giving sufficient time for all bots to spawn on the simulation
  - Waiting until all needed topics are updated before making moves
- Readability and modularity of code
  - Revisit /config for parameter load
  - Generalize /calcs for 3D environment and to robots that are not necessarily Turtlebots
- Account for the potential case of robots coming offline?
- The `.sdf` files that describe the Turtlebot

### How to use
1) Make sure ROS 2 installation is sourced in all terminals you will be using.
   For 'Foxy': `source /opt/ros/foxy/setup.bash`
2) On one terminal, go to the root of the repository, and run `colcon build --symlink-install`
3) On another terminal, source with `. install/setup.bash`
    You can then run the launch file with `ros2 launch multisim multisim.launch.py`
4) On a third terminal, launch Gazebo: `ros2 launch gazebo_ros gazebo.launch.py`