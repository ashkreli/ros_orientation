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
N = 50
I = 10
dt = 1 / I


#with open('pysim/start.yaml') as f:
with open('start.yaml') as f:
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

def plot_performance(s_refs_og, u_refs_og, S_traj, U_traj, S_evol, U_evol):
    s_refs = s_refs_og.copy()
    u_refs = u_refs_og.copy()
    for i in range(len(s_refs_og) - 1):
        for j in range(I):
            # get the next control input vector from the lqr controllers
            noisy_input_traj = lqr.lqr_traj_track_dare(
                S_traj, s_refs, u_refs, dt)[0] + np.array([[np.random.normal(0, 0.05)], 
                                                      [np.random.normal(0, 0.01)]])
            noisy_input_evol = lqr.lqr_evol_ref_dare(
                S_evol, s_refs, dt)[0] + np.array([[np.random.normal(0, 0.05)], 
                                                      [np.random.normal(0, 0.01)]])
            U_traj.append(noisy_input_traj)
            U_evol.append(noisy_input_evol)
            # get the system's state and control input matrices
            A = sim_move.getA()
            noisy_state_traj = S_traj[-1] + np.array([[np.random.normal(0, 0.05)], 
                                            [np.random.normal(0, 0.05)], 
                                            [np.random.normal(0, 0.005)]])
            noisy_state_evol = S_evol[-1] + np.array([[np.random.normal(0, 0.05)], 
                                            [np.random.normal(0, 0.05)], 
                                            [np.random.normal(0, 0.005)]])
            B_traj = sim_move.getB(noisy_state_traj, dt)
            B_evol = sim_move.getB(noisy_state_evol, dt)
            # calculate the next step according to the state space model
            S_traj.append(sim_move.state_space_model(A, noisy_state_traj, B_traj, U_traj[-1]))
            S_evol.append(sim_move.state_space_model(A, noisy_state_evol, B_evol, U_evol[-1]))
        s_refs.pop(0)
        u_refs.pop(0)
    return S_traj, U_traj, S_evol, U_evol

S_spl_traj = [np.array([[x_0], [y_0], [theta_0]])]
U_spl_traj = [np.array([[0],[0]])]
S_spl_evol = [np.array([[x_0], [y_0], [theta_0]])]
U_spl_evol = [np.array([[0],[0]])]
s_refs_og_spl, u_refs_og_spl = gen_s_u(waypts, 20)
S_spl_traj, U_spl_traj, S_spl_evol, U_spl_evol = plot_performance(s_refs_og_spl, u_refs_og_spl, S_spl_traj, U_spl_traj, S_spl_evol, U_spl_evol)

S_cir_traj = [np.array([[x_0], [y_0], [theta_0]])]
U_cir_traj = [np.array([[0],[0]])]
S_cir_evol = [np.array([[x_0], [y_0], [theta_0]])]
U_cir_evol = [np.array([[0],[0]])]
s_refs_og_cir, u_refs_og_cir = trajectory_shapes.circle((-5, 0), 4, 20, 1)
S_cir_traj, U_cir_traj, S_cir_evol, U_cir_evol = plot_performance(s_refs_og_cir, u_refs_og_cir, S_cir_traj, U_cir_traj, S_cir_evol, U_cir_evol)

S_lin_traj = [np.array([[x_0], [y_0], [theta_0]])]
U_lin_traj = [np.array([[0],[0]])]
S_lin_evol = [np.array([[x_0], [y_0], [theta_0]])]
U_lin_evol = [np.array([[0],[0]])]
s_refs_og_lin, u_refs_og_lin = trajectory_shapes.straight_line((2, 4), (10, 23), 20)
S_lin_traj, U_lin_traj, S_lin_evol, U_lin_evol = plot_performance(s_refs_og_lin, u_refs_og_lin, S_lin_traj, U_lin_traj, S_lin_evol, U_lin_evol)

# get coordinates
(x_dare_spl_traj, y_dare_spl_traj) = unpack(S_spl_traj)
(x_dare_spl_evol, y_dare_spl_evol) = unpack(S_spl_evol)
(x_ref_spl, y_ref_spl) = unpack(s_refs_og_spl)

(x_dare_cir_traj, y_dare_cir_traj) = unpack(S_cir_traj)
(x_dare_cir_evol, y_dare_cir_evol) = unpack(S_cir_evol)
(x_ref_cir, y_ref_cir) = unpack(s_refs_og_cir)

(x_dare_lin_traj, y_dare_lin_traj) = unpack(S_lin_traj)
(x_dare_lin_evol, y_dare_lin_evol) = unpack(S_lin_evol)
(x_ref_lin, y_ref_lin) = unpack(s_refs_og_lin)

# plot
fig, ax = plt.subplots()
l1, = ax.plot(x_ref_spl, y_ref_spl, '--og', markersize=2)
l2, = ax.plot(x_dare_spl_traj, y_dare_spl_traj, '--.r', markersize=2)
l3, = ax.plot(x_dare_spl_evol, y_dare_spl_evol, '--.b', markersize=2)

fig, ax1 = plt.subplots()
l4, = ax1.plot(x_ref_cir, y_ref_cir, '--og', markersize=2)
l5, = ax1.plot(x_dare_cir_traj, y_dare_cir_traj, '--.r', markersize=2)
l6, = ax1.plot(x_dare_cir_evol, y_dare_cir_evol, '--.b', markersize=2)

fig, ax2 = plt.subplots()
l7, = ax2.plot(x_ref_lin, y_ref_lin, '--og', markersize=2)
l8, = ax2.plot(x_dare_lin_traj, y_dare_lin_traj, '--.r', markersize=2)
l9, = ax2.plot(x_dare_lin_evol, y_dare_lin_evol, '--.b', markersize=2)

# make plot pretty
ax.legend((l1, l2, l3), ('Reference', 'State with Trajectory Tracking (DARE)', 'State with Evolving Reference Point (DARE)'), loc='upper right', shadow=True)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('Tracking Spline-Interpolated Trajectory (w/ Noise)')

ax1.legend((l4, l5, l6), ('Reference', 'State with Trajectory Tracking (DARE)', 'State with Evolving Reference Point (DARE)'), loc='upper right', shadow=True)
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_title('Tracking Circular Trajectory (w/ Noise)')

ax2.legend((l7, l8, l9), ('Reference', 'State with Trajectory Tracking (DARE)', 'State with Evolving Reference Point (DARE)'), loc='upper right', shadow=True)
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_title('Tracking Linear Trajectory (w/ Noise)')

plt.show()
