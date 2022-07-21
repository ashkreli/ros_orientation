import numpy as np
from trajectory_shapes import straight_line

def test():
    pos_start = (0.0, 0.0)
    pos_end = (4.0, 5.0)
    n = 10
    s_t, u_t = straight_line(pos_start, pos_end, n)
    for i in range(n):
        print(np.array2string(s_t[i]))
    # print(str(u_t))
    print("HI")

test()