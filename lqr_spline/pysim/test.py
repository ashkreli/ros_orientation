from pysim.trajectory_shapes import straight_line, circle
import yaml

import utils.lqr as lqr


import cvxpy as cvx
import numpy as np
import matplotlib.pyplot as plt

def unpack(s):
    x = []
    y = []
    for item in s:
        x.append(item.T[0][0])
        y.append(item.T[0][1])
    return(np.array(x), np.array(y))

def test_yaml():
    pos_start = (0.0, 0.0)
    pos_end = (4.0, 5.0)
    n = 10
    s_t, u_t = straight_line(pos_start, pos_end, n)
    #for i in range(n):
    #    print(np.array2string(s_t[i]))
    # print(str(u_t))
    #print("HI")
    with open('start.yaml') as f:
        my_dict = yaml.safe_load(f)
        print(str(my_dict))

def test_circle():
    center = (5.0, 5.0)
    radius = 2.0
    n = 10
    loops = 5
    s_t, u_t = circle(center, radius, n, 5)
    plt.title("shape") 
    plt.xlabel("x") 
    plt.ylabel("y")
    (x, y) = unpack(s_t)
    plt.plot(x, y, 'bo') 
    #(x_ref, y_ref) = unpack(s_t)
    #plt.plot(x_ref, y_ref, 'go')
    plt.show()

def trying_this_out():
    lookahead = 50
    dt = 0.1
    F = 1.0
    objective = 0
    A = np.array([[1,dt],[0,1]])
    B = np.array([0,dt*F])
    x0 = np.array([1,0])
    xt = cvx.Variable(2)
    state = [xt]
    cost = 0
    constraints = [xt == x0]
    controls = []
    for i in range(lookahead):
        ut = cvx.Variable()
        xtn = cvx.Variable(2)
        controls.append(ut)
        state.append(xtn)

        constraints.append(xtn == A*xt + B * ut )
        constraints.append(ut <= 1.0)   
        constraints.append(ut >= -1.0)  
        cost = cost + cvx.square(xtn[0]) #+ 0.1 * cvx.square(ut)

        xt = xtn

    objective = cvx.Minimize(cost)
    prob = cvx.Problem(objective, constraints)
    sol = prob.solve(verbose=True)
    print(sol)
    pos = np.array(list(map( lambda x: x.value, state)))
    us = np.array(list(map( lambda x: x.value, controls)))

    plt.plot(pos[:,0,0])
    plt.plot(us)
    print(pos[:,0,0])
    plt.show()


s_refs, u_refs = straight_line((1,1), (4,4), 50)
lqr.lqr_traj_track([np.array([[0],[0],[0]])],s_refs,[np.array([[0],[0]])],u_refs,0.1) 
