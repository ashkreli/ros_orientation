import numpy as np

def unpack(S):
    x = []
    y = []
    for item in S:
        x.append(item.T[0][0])
        y.append(item.T[0][1])
    return(np.array(x), np.array(y))