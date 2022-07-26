import numpy as np
import matplotlib.pyplot as plt
import yaml

from plot_utils import unpack

import sys, pathlib, os
path = pathlib.Path(__file__).parent.parent.absolute()
sys.path.append(os.path.join(str(path), 'utils'))
import sim_move, lqr
from spline_int import attach_t, gen_s_u

global N, I, dt
N = 100
I = 2
dt = 1 / I


with open('pysim/start.yaml') as f:
    start = yaml.safe_load(f)
    x_0 = start['x_0']
    y_0 = start['y_0']
    theta_0 = start['theta_0']

waypts = [np.array([[1], [1]]), 
          np.array([[2], [4]]), 
          np.array([[5], [2]]), 
          np.array([[6], [6]])]

max_vels = [0.1, 0.1, 0.1]

waypts = attach_t(waypts, max_vels)

# get the start location
S = [np.array([[x_0], [y_0], [theta_0]])]
U = [np.array([[0],[0]])]
# Generate the discrete trajectory path (the reference)
# s_refs, u_refs = trajectory_shapes.straight_line(pos_start, pos_end, N)
s_refs_og, u_refs_og = gen_s_u(waypts, N)
# copy the refrences
s_refs = s_refs_og.copy()
u_refs = u_refs_og.copy()
for i in range(N - 1):
    for j in range(I):
        # get the next control input vector from the lqr controller
        # print(lqr.lqr_traj_track(S, s_refs, U, u_refs, dt)[1])
        U.append(lqr.lqr_traj_track_dare(S, s_refs, u_refs, dt)[0])
        # get the system's state and control input matrices
        A = sim_move.getA()
        B = sim_move.getB(S[-1], dt)
        # print("Theta " + str(i) + ":" + str((s[-1].T)[0][2]))
        # calculate the next step according to the state space model
        S.append(sim_move.state_space_model(A, S[-1], B, U[-1]))
    s_refs.pop(0)
    u_refs.pop(0)
# get coordinates
(x, y) = unpack(S)
(x_ref, y_ref) = unpack(s_refs_og)
# plot
fig, ax = plt.subplots()
l1, = ax.plot(x_ref, y_ref, '--or')
l2, = ax.plot(x, y, '--og')
plt.annotate('p0', (1,1))
plt.annotate('p1', (2,4))
plt.annotate('p2', (5,2))
plt.annotate('p3', (6,6))
# l3, = ax.plot(x,y, '--ob')
# make plot pretty
ax.legend((l1, l2), ('Reference', 'State with DARE'), loc='upper left', shadow=True)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('Coordinates')
plt.show()