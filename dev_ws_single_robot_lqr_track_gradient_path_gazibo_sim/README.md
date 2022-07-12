## gazebo_multisim_basic

This project is meant to demonstrate various important concepts in ROS2 that can be made use of heavily in simulation and in real robotics systems.
In a completed project, we have how to:
- Launch multiple nodes with a Launch file
- Use .yaml files to configure nodes with parameters
- Make use of Callback groups to handle processes that run in parallel and in series (i.e. managing concurrency)
- Call a service
- Subscribe/publish to topics

Goal: a single robots (in our case TurtleBot3 'burger' models), given a path of references points to reach, must do so with reasonable trajectories. The path mentioned, is generated by gradient decend, and the robot follows the path by using a LQR controller made for this specific case. 

The platform that this code was built on is an Ubuntu 20.04 VM on Windows 11. 
Requirements:
- ROS2 Version 'Foxy' (https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- Gazebo (Follow instructions on https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)
- Python 3.8

The other needed packages should be included in the 'src' directory (tf_transformations).

More in-depth walkthrough of the code is included in the relevant subdirectories for convenience.

### How to use
1) On one terminal, go to the root of the repository, and run `colcon build`
2) On another terminal, source with `. install/setup.bash`
    You can then run the launch file with `ros2 launch multisim multisim.launch.py`
3) On a third terminal, launch Gazebo: `ros2 launch gazebo_ros gazebo.launch.py`