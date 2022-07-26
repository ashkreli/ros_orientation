import numpy as np

def state_space_model(A: np.array, state_t_minus_1: np.array, B: np.array, control_input_t_minus_1: np.array) -> np.array:
    """
    Calculates the state at time t given the state at time t-1, the state matrix,
    the control inputs applied at time t-1, and the control input matrix
    """
    state_estimate_t = (A @ state_t_minus_1) + (B @ control_input_t_minus_1) 
    return state_estimate_t
  
def getB(s: np.array, dt: float) -> np.array:
    """
    Calculates and returns the control input matrix, B matrix, based on the angle
    """
    theta = (s.T)[0][2]
    B =  np.array([[dt * (np.cos(theta)), 0],
                   [dt * (np.sin(theta)), 0],
                   [0,                   dt]])
    return B

def getA() -> np.array:
    """
    Calculates and returns the state matrix, A matrix
    """
    A = np.array([  [1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])
    return A