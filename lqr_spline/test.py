import numpy as np
from trajectory_shapes import straight_line, circle
import yaml
from matplotlib import pyplot as plt 

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

test_circle()