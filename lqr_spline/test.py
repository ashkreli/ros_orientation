from trajectory_shapes import straight_line, circle
#import yaml

import lqr


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


t1 = np.arange(0.0, 2.0, 0.1)
t2 = np.arange(0.0, 2.0, 0.01)

fig, ax = plt.subplots()

# note that plot returns a list of lines.  The "l1, = plot" usage
# extracts the first element of the list into l1 using tuple
# unpacking.  So l1 is a Line2D instance, not a sequence of lines
l1, = ax.plot(t2, np.exp(-t2))
l2, l3 = ax.plot(t2, np.sin(2 * np.pi * t2), '--o', t1, np.log(1 + t1), '.')
l4, = ax.plot(t2, np.exp(-t2) * np.sin(2 * np.pi * t2), 's-.')

ax.legend((l2, l4), ('oscillatory', 'damped'), loc='upper right', shadow=True)
ax.set_xlabel('time')
ax.set_ylabel('volts')
ax.set_title('Damped oscillation')
plt.show()
