import numpy as np
from control import lqr


def lqr_controller(s,s_ref,u_ref,dt):
    """
    Produces the control input vector based on an LQR controller
    """
    # R matrix - The control input cost matrix
    R = np.array([[2,   0],  # Penalty for linear velocity error
                  [0,   0.5]]) # Penalty for angular velocity error
    # Q matrix - The state cost matrix.
    Q = np.array([[2, 0, 0],  # Penalize X position error 
                  [0, 2, 0],  # Penalize Y position error 
                  [0, 0, 1]]) # Penalize ANGLE heading error  

    # calculate the control and control input matrices of the system
    A_tilde = getAtilde(s_ref, u_ref, dt)
    B_tilde = getBtilde(s_ref, dt)

    K, S, E = lqr(A_tilde, B_tilde, Q, R)

    #print(K)
    #print(s-s_ref)
    # get the control input vector as calculated by the LQR controller
    # JUST for the next steps to maintain a closed loop control 
    u = (-K @ (s-s_ref) + u_ref)

    return u

def getAtilde(s_ref, u_ref, dt):
    """
    Calculates and returns the state matrix
    """
    A_tilde = np.array([[1, 0, (-dt * u_ref[0] * np.sin(s_ref[2]))[0]],
                        [0, 1,  (dt * u_ref[0] * np.cos(s_ref[2]))[0]],
                        [0, 0,  1                                    ]])
    return A_tilde

def getBtilde(s_ref, dt):
    """
    Calculates and returns the control input matrix,
    """
    B_tilde = np.array([[(dt * np.cos(s_ref[2]))[0], 0],
                        [(dt * np.sin(s_ref[2]))[0], 0],
                        [0,                         dt]])
    return B_tilde