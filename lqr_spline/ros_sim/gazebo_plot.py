from typing import List
import numpy as np
import matplotlib.pyplot as plt

import sys, pathlib, os
path = pathlib.Path(__file__).absolute().parent.parent
print(str(path))
path = os.path.join(path, 'utils')
sys.path.append(str(path))
from plot_utils import unpack, parse_into_np



# Parse the ideal and actually traversed (x, y, theta) poses
states = parse_into_np('states.txt', 3)
x_real, y_real = unpack(states)
refs = parse_into_np('refs.txt', 3)
x_refs, y_refs = unpack(refs)

# Parse the ideal and actually traversed control inputs
u_ideal = parse_into_np('inputs_ideal.txt', 2)
v_ideal, omega_ideal = unpack(u_ideal)
u_actual = parse_into_np('inputs_actual.txt', 2)
v_actual, omega_actual = unpack(u_actual)

# plot
fig, ax = plt.subplots()
l1, = ax.plot(x_refs, y_refs, '--og', markersize=2)
l2, = ax.plot(x_real, y_real, '--.r', markersize=2)

ax.legend((l1, l2), ('Reference', 'Simulation'), loc='upper right', shadow=True)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('Gazebo Simulation')

fig, ax1 = plt.subplots()
l3, = ax1.plot(v_ideal, omega_ideal, '--og', markersize=2)
l4, = ax1.plot(v_actual, omega_actual, '--r', markersize=2)

ax1.legend((l3, l4), ('Reference', 'Simulation'), loc='upper right', shadow=True)
ax1.set_xlabel('v')
ax1.set_ylabel('omega')
ax1.set_title('Gazebo Simulation')

plt.show()