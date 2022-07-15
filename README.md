# ROS2 Orientation

### Arba Shkreli and Elianne Sacher
### Harvard School of Engineering and Applied Sciences
### Summer 2022

## Introduction

This repository holds a series of small projects in our process of getting
familiar with ROS (Robot Operating System) 2 for use in applying the control
methods of PID and LQR in single point reference tracking with one or more 
robots, as well as path trajectory tracking.

Many of these projects build on each other, so much of the setup code repeats
itself. `gazebo_multisim_basic` was the project where the majority of the ROS 2
features were figured out for later projects, so that the focus would shift to
more control-related problems. Separate directories do not necessarily mean distinct
projects, but the code is self-contained in each. Documentation is found in each
folder and a detailed explanation of our work can be found in our reports.

## Folders

- turtlesim_pid_ctrl (Elianne)
    - A starter project to better understand ROS concepts
    - Have one Turtlebot go to a predefined reference point using PID velocity
        tracking
    - Thorough step-by-step guide for anyone just getting started
- gazebo_multisim_basic (Arba)
    - Control of multiple robots on a more sophisticated simulation platform,
        Gazebo, to approach a common reference point without colliding
- gazebo_single_lqr_gradient_path (Elianne)
    - ()
- gazebo_multi_lqr_gradient_path (Elianne)
    - ()
- gazebo_lqr_path_tracking (Arba - in progress)
    - Building upon Elianne's work with LQR waypoint tracking, this extension of
        the project seeks to test out whether LQR can reasonably follow a given
        path

## Contact

| Arba Shkreli                        | Elianne Sacher                           |
| ----------------------------------- | ---------------------------------------- |
| Email: ashkreli@college.harvard.edu | Email: eliannesacher@college.harvard.edu |
| GitHub: ashkreli                    | GitHub: elianne-sacher                   |
