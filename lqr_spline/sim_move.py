import numpy as np

# Optional Variables
global max_linear_velocity, max_angular_velocity
max_linear_velocity = 3.0 # meters per second
max_angular_velocity = 0.5 # radians per second

def getNextS(s,u):
    """
    returns the next state for a given state and control input
    """
    # get the system's state and control input matrices
    A = getA()
    B = getB((s.T)[0][2], dt)
    next_s = sim_move.state_space_model(A, s, B, u)
    return next_s
    
    return next_s
def state_space_model(A, state_t_minus_1, B, control_input_t_minus_1):
    """
    Calculates the state at time t given the state at time t-1, the state matrix,
    the control inputs applied at time t-1, and the control input matrix
    """
    # These next 6 lines of code which place limits on the angular and linear 
    # velocities of the robot car can be removed if desired.
    control_input_t_minus_1[0] = np.clip(control_input_t_minus_1[0],
                                         -max_linear_velocity,
                                         max_linear_velocity)
    control_input_t_minus_1[1] = np.clip(control_input_t_minus_1[1],
                                         -max_angular_velocity,
                                         max_angular_velocity)

    state_estimate_t = (state_t_minus_1) + (B @ control_input_t_minus_1) 
    return state_estimate_t
  
def getB(theta, dt):
    """
    Calculates and returns the control input matrix, B matrix, based on the angle
    """
    B =  np.array([[dt * (np.cos(theta)), 0],
                   [dt * (np.sin(theta)), 0],
                   [0,                   dt]])
    return B

def getA():
    """
    Calculates and returns the state matrix, A matrix
    """
    A = np.array([  [1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])
    return A