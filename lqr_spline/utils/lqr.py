from typing import List
import numpy as np
from control import lqr
import cvxpy as cvx

import sim_move

def lqr_traj_track_cvxpy(S: List[np.array],s_refs: List[np.array],u_refs: List[np.array],dt: float) -> List[np.array]:
    """
    Produces the control input vector based on an LQR controller
    for the next s_refs and u_refs recieved as input
    This function utilizes CVXPY to solve for the controller.
    """

    # R matrix - The control input cost matrix
    R = np.array([[2,   0],  # Penalty for linear velocity error
                  [0,   0.5]]) # Penalty for angular velocity error
    # Q matrix - The state cost matrix.
    Q = np.array([[2, 0, 0],  # Penalize X position error 
                  [0, 2, 0],  # Penalize Y position error 
                  [0, 0, 1]]) # Penalize ANGLE heading error  
    

    # get the list of state and control input matrices
    A_tildes = getAtildes(s_refs, u_refs, dt)
    B_tildes = getBtildes(s_refs, dt)
    
    # setup the variables for the opt. prob.
    u_list = []
    s_list = []
    for i in range(len(s_refs)):
        s_list.append(cvx.Variable((3,1)))
        u_list.append(cvx.Variable((2,1)))

    # apply the constraints
    constraints = [(s_list[0] == S[-1])]

    # get the performance index
    prf_inx = cvx.quad_form((s_list[0] - s_refs[0]),Q) + cvx.quad_form((u_list[0] - u_refs[0]),R)
    
    for t in range(1,len(s_refs)):
        
        # add the relavent constraint to the opt. question
        constraints.append(s_list[t] - s_refs[t] == A_tildes[t] @ (s_list[t-1]-s_refs[t-1]) + B_tildes[t] @ (u_list[t-1]-u_refs[t]))

        # calculate the performance index
        prf_inx += cvx.quad_form((s_list[t] - s_refs[t]),Q) + cvx.quad_form((u_list[t] - u_refs[t]),R)

    # get the objective of the opt. question
    objective = cvx.Minimize(prf_inx)

    # define the problem and solve it
    prob = cvx.Problem(objective, constraints)
    prob.solve()

    # return the calculated control inputs 
    U = []
    for i in range(len(u_list)):
        U.append(u_list[i].value)
    return U

def lqr_traj_track_dare(S: List[np.array],s_refs: List[np.array],u_refs: List[np.array],dt: float) -> List[np.array]:
    """
    Produces the control input vector based on an LQR controller
    for the next s_refs and u_refs recieved as input.
    This function uses DARE to find the controller.
    """
    
    # R matrix - The control input cost matrix
    R = np.array([[2,   0],  # Penalty for linear velocity error
                  [0,   0.5]]) # Penalty for angular velocity error
    # Q matrix - The state cost matrix.
    Q = np.array([[2, 0, 0],  # Penalize X position error 
                  [0, 2, 0],  # Penalize Y position error 
                  [0, 0, 1]]) # Penalize ANGLE heading error  
    

    # get the list of state and control input matrices
    A_tildes = getAtildes(s_refs, u_refs, dt)
    B_tildes = getBtildes(s_refs, dt)
    
    # Solutions to discrete LQR problems are obtained using the dynamic 
    # programming method.
    # The optimal solution is obtained recursively, starting at the last 
    # timestep and working backwards.
    N = len(s_refs) - 1
 
    # Create a list of N + 1 elements
    P = [None] * (N + 1)
      
    # LQR via Dynamic Programming
    P[N] = Q
 
    # For i = N, ..., 1
    for i in range(N, 0, -1):
        # Discrete-time Algebraic Riccati equation to calculate the optimal 
        # state cost matrix
        P[i-1] = Q + A_tildes[i].T @ P[i] @ A_tildes[i] - (A_tildes[i].T @ P[i] @ B_tildes[i]) @ np.linalg.pinv(
            R + B_tildes[i].T @ P[i] @ B_tildes[i]) @ (B_tildes[i].T @ P[i] @ A_tildes[i])      
    
    # Create a list of N elements
    K = [None] * N
    u = [None] * N
    
    # new S assuming no noise and open loop
    new_s = [S[-1]]
    # For i = 0, ..., N - 1
    for i in range(N):
        # Calculate the optimal feedback gain K
        K[i] = np.linalg.pinv(R + B_tildes[i].T @ P[i+1] @ B_tildes[i]) @ B_tildes[i].T @ P[i+1] @ A_tildes[i]
        u[i] = - K[i] @ (new_s[i] - s_refs[i]) + u_refs[i]
        new_s.append(sim_move.state_space_model(sim_move.getA(), new_s[i], sim_move.getB(new_s[i],dt), u[i]))
    return u

def getAtildes(s_refs: List[np.array], u_refs: List[np.array], dt: float) -> List[np.array]:
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

def getBtildes(s_refs: List[np.array], dt: float) -> List[np.array]:
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

def getBhat(s: np.array, dt: float) -> np.array:
    """
    Calculates and returns the control input matrix for the point system
    """
    theta = (s.T)[0][2]
    B =  np.array([[dt * (np.cos(theta)), 0],
                   [dt * (np.sin(theta)), 0],
                   [0,                   dt]])
    return B

def getAhat() -> np.array:
    """
    Calculates and returns the state matrix, A matrix
    """
    A = np.array([  [1.0, 0, 0],
                    [0, 1.0, 0],
                    [0, 0, 1.0]])
    return A