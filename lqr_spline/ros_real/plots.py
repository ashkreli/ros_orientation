import matplotlib.pyplot as plt

import sys, pathlib, os
path = pathlib.Path(__file__).absolute().parent.parent
print(str(path))
path = os.path.join(path, 'utils')
sys.path.append(str(path))
from plot_utils import unpack, parse_into_np


# Parse reference and actually traversed states
states = parse_into_np('states.txt', 3)
x_real, y_real = unpack(states)
refs = parse_into_np('refs.txt', 3)
x_refs, y_refs = unpack(refs)
actual_inputs = parse_into_np('actual_inputs.txt', 2)
v_actual, omega_actual = unpack(actual_inputs)

# plot
fig, ax = plt.subplots()
l1, = ax.plot(x_refs, y_refs, '--og', markersize=2)
l2, = ax.plot(x_real, y_real, '--.r', markersize=2)

ax.legend((l1, l2), ('Reference', 'Real'), loc='upper right', shadow=True)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('Real Turtlebot')

plt.show()