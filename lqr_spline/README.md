## lqr_spline
#### Collaborators: Arba Shkreli and Elianne Sacher

Testing our algorithm involved going through multiple, progressively more realistic
settings. First, starting with the absolutely ideal Python simulation which basically
treats the robot as a point mass capable of going as fast as desired, to a real
Turtlebot which cannot make control calculations extremely often, and has significant
limitations on angular and linear speeds.

`pysim`
- Python simulation of system given a handful of trajectories
    - System evolution modeled with the state-space, simple Newtonian model
- System performance studied with actuator and measurement noise
- Additionally, we studied the difference in performance between implementing
the LQR Trajectory Tracking with DARE or with CVXPY problem formulation and
optimization

`ros_sim`
- Gazebo ROS 2 simulation of the system given the same trajectories
- It is closer to understanding how our controller behaves with real robots because
the simulated robot has physical properties based on its shape
- Introduces the real problems of control input realizability given physical constraints

`ros_real`
- Implementation on a real Turtlebot3 with ROS 2
- Physical constraints are more greatly accentuated than on Gazebo
- What remains ideal about this setup is the fact that a Turtlebot knows
its coordinates by the external, highly accurate OptiTrack system

Each of these stages have associated Python-generated plots to better visualize
the extent to which the systems follow their given trajectories. See documentation
within each subfolder to understand how to generate the plots.

`utils`
- Contains all the repeated code across the different testing environments
- Utilities for making plotting easier
- LQR and Spline Interpolation code
