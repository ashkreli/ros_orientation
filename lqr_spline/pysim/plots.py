import yaml
import matplotlib.pyplot as plt
import numpy as np
import time

import trajectory_shapes
from plot_utils import unpack

import sys, pathlib, os
path = pathlib.Path(__file__).parent.parent.absolute()
sys.path.append(os.path.join(str(path), 'utils'))
import lqr, sim_move

global I
I = 15

global dt
dt = 1 / I # s

global N
N = 50
 # a large enough number

# Unload starting params from start.yaml
with open('pysim/start.yaml') as f:
    start = yaml.safe_load(f)
    x_0 = start['x_0']
    y_0 = start['y_0']
    theta_0 = start['theta_0']
    x_line_0 = start['x_line_0']
    y_line_0 = start['y_line_0']
    x_line_N = start['x_line_N']
    y_line_N = start['y_line_N']

global s_0
s_0 = np.array([[x_0],[y_0],[theta_0]])

global pos_start, pos_end
pos_start = (x_line_0, y_line_0)
pos_end = (x_line_N, y_line_N)


def main():
    # check time differences between the DARE LQR and the CVXPY LQR
    t_CVXPY = []
    t_DARE = []
    # get the start location
    S_cvxpy = [s_0]
    S_dare = [s_0]
    U_cvxpy = [np.array([[0],[0]])]
    U_dare = [np.array([[0],[0]])]
    # Generate the discrete trajectory path (the reference)
    ''' uncomment the following line for recieving the straight line trajectory path'''
    #s_refs_og, u_refs_og = trajectory_shapes.straight_line(pos_start, pos_end, N) 
    ''' uncomment the following line for recieving the circular trajectory path'''
    s_refs_og, u_refs_og = trajectory_shapes.circle((0.0, 0.0), 4.0, N, 1)
    # copy the refrences
    s_refs = s_refs_og.copy()
    u_refs = u_refs_og.copy()
    for i in range(len(s_refs)-1):
        for j in range(I):
            # get the next control input vector from the lqr controller
            t0 = time.perf_counter()
            ''' uncomment the following line for recieving the trajectory tracking cvxpy control input'''
            U_cvxpy.append(lqr.lqr_traj_track_cvxpy(S_cvxpy,s_refs,u_refs,dt)[0])
            ''' uncomment the following line for recieving the evolving refrence point tracking cvxpy control input'''
            #U_cvxpy.append(lqr.lqr_evol_ref_cvxpy(S_cvxpy,s_refs,dt)[0])
            t1 = time.perf_counter()
            ''' uncomment the following line for recieving the trajectory tracking DARE control input'''
            U_dare.append(lqr.lqr_traj_track_dare(S_dare, s_refs, u_refs, dt)[0])
            ''' uncomment the following line for recieving the evolving refrence point tracking DARE control input'''
            #U_dare.append(lqr.lqr_evol_ref_dare(S_dare, s_refs, dt)[0])
            t2 = time.perf_counter()
            t_CVXPY.append(t1-t0)
            t_DARE.append(t2-t1)
            # get the system's state and control input matrices
            A = sim_move.getA()
            B_cvxpy = sim_move.getB(S_cvxpy[-1], dt)
            B_dare = sim_move.getB(S_dare[-1], dt)
            # calculate the next step according to the state space model
            S_cvxpy.append(sim_move.state_space_model(A, S_cvxpy[-1], B_cvxpy, U_cvxpy[-1]))
            S_dare.append(sim_move.state_space_model(A, S_dare[-1], B_dare, U_dare[-1]))
        s_refs.pop(0)
        u_refs.pop(0)
    
  
    # get coordinates
    (x_cvxpy, y_cvxpy) = unpack(S_cvxpy)
    (x_dare, y_dare) = unpack(S_dare)
    (x_ref, y_ref) = unpack(s_refs_og)
    
    # plot
    fig, ax = plt.subplots()
    l2, = ax.plot(x_cvxpy, y_cvxpy, '--og',markersize=10)
    l3, = ax.plot(x_dare,y_dare, '--.b')
    l1, = ax.plot(x_ref, y_ref, '--or')
    
    # make plot pretty
    ax.legend((l1, l2, l3), ('Reference', 'State with CVXPY', 'State with DARE'), loc='upper right', shadow=True)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_title('Line')
    
    # plot
    fig, ax1 = plt.subplots()
    l4, = ax1.plot(np.array(range(1,len(t_CVXPY)+1)), np.array(t_CVXPY), 'r')
    l5, = ax1.plot(np.array(range(1,len(t_DARE)+1)), np.array(t_DARE), 'g')
    
    # make plot pretty
    ax1.legend((l4, l5), ('RUNTIME with CVXPY', 'RUNTIME with DARE'), loc='upper right', shadow=True)
    ax1.set_xlabel('Run Count [1]')
    ax1.set_ylabel('Run Time [s]')
    ax1.set_title('Line')
    plt.show()

main()
