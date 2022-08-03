The `utils` folder contains the implementations of the core concepts of the project
involving the use of LQR to follow a given trajectory. The theoretical aspects
of our work are elaborated in our reports.

`lqr.py`:
- Two code implementations of the LQR Trajectory Tracking Method
    - Using convex optimization package CVXPY
    - Solving Discrete Algebraic Ricatti Equation (DARE)
- One code implementation of the LQR Evolving Reference Point Method

`sim_move.py`:
- State-space model for our dynamical system
    - A and B matrices

`spline_int.py`:
- Parametrized by time curve fitting of discrete ordered waypoints provided by
the user

`calcs.py`:
- Functions frequently necessary all over
- Note on `state_error()`: important to use to find the true error between states,
because the angular error is not always current_angle - reference_angle, since we want
the smallest angle between them, and whether the reference is CW or CCW from the current
angle.

`plot_utils.py`:
- Utilities to plot the states of the system, whether reference or actually traversed
- Currently, there is only one function `unpack`

`trajectory_shapes.py`:
- Generic test trajectories for our robot to track
    - A CCW circle with constant speed
    - A straight line with constant speed