import numpy as np
from control import lqr
import cvxpy as cvx


def lqr_traj_track(S,s_refs,U,u_refs,dt):
    """
    Produces the control input vector based on an LQR controller
    """
    # coppying lists
    s_refs = s_refs.copy()
    u_refs = u_refs.copy()

    # R matrix - The control input cost matrix
    R = np.array([[2,   0],  # Penalty for linear velocity error
                  [0,   0.5]]) # Penalty for angular velocity error
    # Q matrix - The state cost matrix.
    Q = np.array([[2, 0, 0],  # Penalize X position error 
                  [0, 2, 0],  # Penalize Y position error 
                  [0, 0, 1]]) # Penalize ANGLE heading error  
    

    # get the list of state and control input matrices
    A_tildes = getAtilde(s_refs, u_refs, dt)
    B_tildes = getBtilde(s_refs, dt)

    # setup initial cond. for opt. prob.
    s_delta_0 = S[-1] - s_refs[len(S)-1]
    u_delta_0 = U[-1] - u_refs[len(U)-1]

    # setup the variables for the opt. prob.
    s_delta = cvx.Variable((3,1),name="s_delta_t")
    u_delta = cvx.Variable((2,1),name="u_delta_t")

    # apply the constraints
    constraints = [(s_delta == s_delta_0),(u_delta == u_delta_0)]

    # instantiate the list of control inputs
    us_delta = []

    # get the performance index
    prf_inx = 0
    for t in range(len(S)-1,len(s_refs)):
        s_delta_t = cvx.Variable((3,1),name="s_delta_t")
        u_delta_t = cvx.Variable((2,1),name="u_delta_t")

        us_delta.append(u_delta_t)

        # add the relavent constraint to the opt. question
        constraints.append(s_delta_t == A_tildes[t] @ s_delta + B_tildes[t] @ u_delta)

        # calculate the performance index
        prf_inx += (s_delta_t.T @ Q @ s_delta_t + u_delta_t.T @ R @ u_delta_t)[0][0]
        
        # set current state vector and control input to the calculated next one
        s_delta = s_delta_t
        u_delta = u_delta_t
    
    # get the objective of the opt. question
    objective = cvx.Minimize(prf_inx)

    # define the problem and solve it
    prob = cvx.Problem(objective, constraints)
    sol = prob.solve(verbose=True)

    # get and return the list of control inpots
    new_U = np.array(list(map( lambda x: x.value, us_delta + u_refs[(len(S)-1) :])))
    print(new_U)
    return new_U

def getAtilde(s_refs, u_refs, dt):
    """
    Calculates and returns the state matrices for the delta system
    for ALL refrences
    """
    A_tildes = []
    for (s_ref,u_ref) in zip(s_refs,u_refs):
        A_tildes.append(np.array([[1, 0, (-dt * u_ref[0] * np.sin(s_ref[2]))[0]],
                                  [0, 1,  (dt * u_ref[0] * np.cos(s_ref[2]))[0]],
                                  [0, 0,  1                                    ]]))
    return A_tildes

def getBtilde(s_refs, dt):
    """
    Calculates and returns the control input matrices for the delta system
    for ALL refrences
    """
    B_tildes = []
    for s_ref in s_refs:
        B_tildes.append(np.array([[(dt * np.cos(s_ref[2]))[0], 0],
                            [(dt * np.sin(s_ref[2]))[0], 0],
                            [0,                         dt]]))
    return B_tildes

def getBhat(s, dt):
    """
    Calculates and returns the control input matrix for the point system
    """
    theta = (s.T)[0][2]
    B =  np.array([[dt * (np.cos(theta)), 0],
                   [dt * (np.sin(theta)), 0],
                   [0,                   dt]])
    return B

def getAhat():
    """
    Calculates and returns the state matrix, A matrix
    """
    A = np.array([  [1.0, 0, 0],
                    [0, 1.0, 0],
                    [0, 0, 1.0]])
    return A