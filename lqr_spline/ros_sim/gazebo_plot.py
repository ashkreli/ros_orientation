from typing import List
from itertools import islice
import numpy as np
import matplotlib.pyplot as plt

import sys, pathlib, os
path = pathlib.Path(__file__).absolute().parent.parent
print(str(path))
path = os.path.join(path, 'utils')
sys.path.append(str(path))
from plot_utils import unpack

# TODO: parse states.txt into np arrays
# TODO: parse refs.txt into np arrays

def parse_into_np(filename: str, elt_num) -> List[np.array]:
    array_list = []
    with open(filename, 'r') as f:
        array_elts = []
        for line in f.readlines():
            if len(array_elts) == elt_num:
                array_list.append(np.array([[array_elts[0]],[array_elts[1]],[array_elts[2]]]))
                array_elts = []
            array_elts.append(np.float(line.strip().replace('[', '').replace(']', '')))
    return array_list

states = parse_into_np('states.txt', 3)
x_real, y_real = unpack(states)
refs = parse_into_np('refs.txt', 3)
x_refs, y_refs = unpack(refs)

# plot
fig, ax = plt.subplots()
l1, = ax.plot(x_refs, y_refs, '--og', markersize=2)
l2, = ax.plot(x_real, y_real, '--.r', markersize=2)

ax.legend((l1, l2), ('Reference', 'Simulation'), loc='upper right', shadow=True)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('Gazebo Simulation')

plt.show()