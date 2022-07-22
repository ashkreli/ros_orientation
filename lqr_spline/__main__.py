from re import M
import numpy as np
import yaml
from matplotlib import pyplot as plt 

import lqr
import sim_move
import trajectory_shapes

global I
I = 20

global dt
dt = 1 / I # s

global N
N = 10
 # a large enough number

with open('start.yaml') as f:
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
    # get the start location
    s_traj = [s_0]
    s_evol = [s_0]
    # Generate the discrete trajectory path (the reference)
    s_refs, u_refs = trajectory_shapes.straight_line(pos_start, pos_end, N) 
    # s_refs, u_refs = trajectory_shapes.circle((0.0, 0.0), 4.0, N, loops)
    for i in range(N):
        for j in range(I):
            # lqr trajectory tracking
            u_traj = lqr_traj_traking(s_traj[-1],s_refs[i],u_refs[i],dt)
            # lqr evolving reference point tracking
            u_evol = lqr_evol_ref_traking(s_evol[-1],s_refs[i],dt)

            # simulate movement for traj track
            s_traj.append(sim_move.getNextS(s_traj[-1],u_traj))
            # simulate movement for ref point track
            s_evol.append(sim_move.getNextS(s_evol[-1],u_evol))
            
    # plot
    plt.title("coordinates") 
    plt.xlabel("x") 
    plt.ylabel("y")
    # plot the reference waypoints
    (x_ref, y_ref) = unpack(s_refs)
    plt.plot(x_ref, y_ref, 'go')
    # plot the simulated waypoints from the traj track method
    (x_traj, y_traj) = unpack(s_traj)
    plt.plot(x_traj, y_traj, 'bo') 
    # plot the simulated waypoints from the evol ref point track method
    (x_evol, y_evol) = unpack(s_evol)
    plt.plot(x_evol, y_evol, 'ro')
    plt.show()

def lqr_traj_traking(s,s_ref,u_ref,dt):
    """
    Generate control with the LQR trajectory tracking method
    """
    # calculate the control and control input matrices of the delta system
    A_tilde = lqr.getAtilde(s_ref, u_ref, dt)
    B_tilde = lqr.getBtilde(s_ref, dt)
    # get the next control input vector from the lqr controller
    u = lqr.lqr_controller(A_tilde, B_tilde)
    
    return u

def lqr_evol_ref_traking(s,s_ref,dt):
    """
    Generate control with the LQR evolving reference point tracking method
    """    
    # calculate the control and control input matrices of the delta system
    A_hat = lqr.getAhat()
    B_hat = lqr.getBhat(s,dt)
    # get the next control input vector from the lqr controller
    K = lqr.lqr_controller(A_hat, B_hat)
    # get the control input vector as calculated by the LQR controller
    # JUST for the next steps to maintain a closed loop control 
    u = (-K @ (s- s_ref))

    return u



def unpack(s):
    x = []
    y = []
    for item in s:
        x.append(item.T[0][0])
        y.append(item.T[0][1])
    return(np.array(x), np.array(y))

if __name__ == '__main__':
    main()