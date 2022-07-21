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
    s = [s_0]
    loops = 5
    # Generate the discrete trajectory path (the reference)
    s_refs, u_refs = trajectory_shapes.straight_line(pos_start, pos_end, N) 
    # s_refs, u_refs = trajectory_shapes.circle((0.0, 0.0), 4.0, N, loops)
    for i in range(N):
        for j in range(I):
            # get the next control input vector from the lqr controller
            u = lqr.lqr_controller(s[-1], s_refs[i], u_refs[i], dt)
            # get the system's state and control input matrices
            A = sim_move.getA()
            B = sim_move.getB((s[-1].T)[0][2], dt)
            # print("Theta " + str(i) + ":" + str((s[-1].T)[0][2]))
            # calculate the next step according to the state space model
            s.append(sim_move.state_space_model(A, s[-1], B, u))
    plt.title("coordinates") 
    plt.xlabel("x") 
    plt.ylabel("y")
    (x, y) = unpack(s)
    plt.plot(x, y, 'bo') 
    (x_ref, y_ref) = unpack(s_refs)
    plt.plot(x_ref, y_ref, 'go')
    plt.show()

def unpack(s):
    x = []
    y = []
    for item in s:
        x.append(item.T[0][0])
        y.append(item.T[0][1])
    return(np.array(x), np.array(y))

if __name__ == '__main__':
    main()