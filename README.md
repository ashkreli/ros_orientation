# ROS2 Orientation

### Arba Shkreli and Elianne Sacher
### Harvard School of Engineering and Applied Sciences
### Summer 2022

## Introduction

This repository holds a series of small projects in our process of getting
familiar with ROS (Robot Operating System) 2 for use in applying the control
methods of PID and LQR in single point reference tracking with one or more 
robots, as well as path trajectory tracking. The capstone is `lqr_spline`.

Many of these projects build on each other, so much of the setup code repeats
itself. `gazebo_multisim_basic` was the project where the majority of the ROS 2
features were figured out for later projects, so that the focus would shift to
more control-related questions. Separate directories do not necessarily mean distinct
projects, but the code is self-contained in each. Documentation is found in each
folder and a detailed explanation of our work can be found in our reports.

## Folders

- turtlesim_pid_ctrl (Elianne)
    - A starter project to better understand ROS concepts
    - Have one Turtlebot go to a predefined reference point using PID velocity tracking
    - Thorough step-by-step guide for anyone just getting started
- gazebo_multisim_basic (Arba)
    - Control of multiple robots on a more sophisticated simulation platform,
        Gazebo, to approach a common reference point without colliding
- real_single_lqr_gradient_path (Elianne)
    - Control of a single robot on a actual turtlebots
        to approach a common reference point without colliding by using an LQR controller
- gazebo_single_lqr_gradient_path (Elianne)
    - Control of a single robot on a more sophisticated simulation platform,
        Gazebo, to approach a common reference point without colliding by using an LQR controller
- gazebo_multi_lqr_gradient_path (Elianne)
    - Control of multiple robots on a more sophisticated simulation platform,
        Gazebo, to approach a common reference point without colliding by using an LQR controller
- lqr_spline (Arba, Elianne)
    - Testing two LQR-based methods of tracking a given path under progressively more realistic settings. First a Python simulation, then a Gazebo simulation, and then lastly a real Turtlebot3
        - LQR Reference Tracking Method: Define trajectory as a set of points for each discrete time step $t_i$ $(x(t_i), y(t_t))$ and control inputs $(v(t_i), \omega(t_i))$ to ideally track those waypoints assuming the robot starts on the path. The true control input is calculated and applied multiple times between each waypoint within the time interval $(t_i, t_{i+1})$ such that the cost of trajectory
        error is minimized over the remaining time steps (solving the LQ problem).
        - LQR Evolving Reference Point Method: The trajectory is not made up of ideal control inputs, and the system is linearized 
    - Spline Interpolation: Generating time-parameterized trajectories, which include ideal control inputs, by least-squares third-degree polynomial curve fitting of user-given discrete but ordered positions to track.

## Reports
- Reference Point Tracking
- LQR Trajectory Tracking Methods

## Contact

| Arba Shkreli                        | Elianne Sacher                           |
| ----------------------------------- | ---------------------------------------- |
| Email: ashkreli@college.harvard.edu | Email: eliannesacher@college.harvard.edu |
| GitHub: ashkreli                    | GitHub: elianne-sacher                   |
