import numpy as np
import matplotlib.pyplot as plt
import yaml

from plot_utils import unpack
import trajectory_shapes

import sys, pathlib, os
path = pathlib.Path(__file__).parent.parent.absolute()
sys.path.append(os.path.join(str(path), 'utils'))
import sim_move, lqr
from spline_int import attach_t, gen_s_u


global N, I, dt
N = 20
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

def plot_performance(s_refs_og, u_refs_og, S, U):
    s_refs = s_refs_og.copy()
    u_refs = u_refs_og.copy()
    for i in range(len(s_refs_og) - 1):
        for j in range(I):
            # get the next control input vector from the lqr controller
            noisy_input = lqr.lqr_traj_track_dare(
                S, s_refs, u_refs, dt)[0] + np.array([[np.random.normal(0, 0.05)], 
                                                      [np.random.normal(0, 0.01)]])
            U.append(noisy_input)
            # get the system's state and control input matrices
            A = sim_move.getA()
            noisy_state = S[-1] + np.array([[np.random.normal(0, 0.05)], 
                                            [np.random.normal(0, 0.05)], 
                                            [np.random.normal(0, 0.005)]])
            B = sim_move.getB(noisy_state, dt)
            # calculate the next step according to the state space model
            S.append(sim_move.state_space_model(A, noisy_state, B, U[-1]))
        s_refs.pop(0)
        u_refs.pop(0)
    return S, U

S_spl = [np.array([[x_0], [y_0], [theta_0]])]
U_spl = [np.array([[0],[0]])]
s_refs_og_spl, u_refs_og_spl = gen_s_u(waypts, 20)
S_spl, U_spl = plot_performance(s_refs_og_spl, u_refs_og_spl, S_spl, U_spl)

S_cir = [np.array([[x_0], [y_0], [theta_0]])]
U_cir = [np.array([[0],[0]])]
s_refs_og_cir, u_refs_og_cir = trajectory_shapes.circle((-5, 0), 4, 20, 1)
S_cir, U_cir = plot_performance(s_refs_og_cir, u_refs_og_cir, S_cir, U_cir)

S_lin = [np.array([[x_0], [y_0], [theta_0]])]
U_lin = [np.array([[0],[0]])]
s_refs_og_lin, u_refs_og_lin = trajectory_shapes.straight_line((2, 4), (10, 23), 20)
S_lin, U_lin = plot_performance(s_refs_og_lin, u_refs_og_lin, S_lin, U_lin)


# get coordinates
(x_dare_spl, y_dare_spl) = unpack(S_spl)
(x_ref_spl, y_ref_spl) = unpack(s_refs_og_spl)

(x_dare_cir, y_dare_cir) = unpack(S_cir)
(x_ref_cir, y_ref_cir) = unpack(s_refs_og_cir)

(x_dare_lin, y_dare_lin) = unpack(S_lin)
(x_ref_lin, y_ref_lin) = unpack(s_refs_og_lin)

# plot
fig, ax = plt.subplots()
l1, = ax.plot(x_ref_spl, y_ref_spl, '--og', markersize=2)
l2, = ax.plot(x_dare_spl, y_dare_spl, '--.b', markersize=2)

fig, ax1 = plt.subplots()
l3, = ax1.plot(x_ref_cir, y_ref_cir, '--og', markersize=2)
l4, = ax1.plot(x_dare_cir, y_dare_cir, '--.b', markersize=2)

fig, ax2 = plt.subplots()
l5, = ax2.plot(x_ref_lin, y_ref_lin, '--og', markersize=2)
l6, = ax2.plot(x_dare_lin, y_dare_lin, '--.b', markersize=2)

# make plot pretty
ax.legend((l1, l2), ('Reference', 'State with DARE'), loc='upper left', shadow=True)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('Tracking Spline-Interpolated Trajectory (w/ Noise)')

ax1.legend((l3, l4), ('Reference', 'State with DARE'), loc='upper left', shadow=True)
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_title('Tracking Circular Trajectory (w/ Noise)')

ax2.legend((l5, l6), ('Reference', 'State with DARE'), loc='upper left', shadow=True)
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_title('Tracking Linear Trajectory (w/ Noise)')

plt.show()