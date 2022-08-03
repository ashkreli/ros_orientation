import numpy as np
from typing import List


def unpack(S):
    x = []
    y = []
    for item in S:
        x.append(item.T[0][0])
        y.append(item.T[0][1])
    return(np.array(x), np.array(y))


def parse_into_np(filename: str, elt_num: int) -> List[np.array]:
    """ Assuming file just has (elt_num, 1) vector numpy arrays written,
        parse the file into a list of numpy arrays """
    array_list = []
    with open(filename, 'r') as f:
        array_elts = []
        for line in f.readlines():
            if len(array_elts) == elt_num:
                array_list.append(np.array([[array_elts[i]] for i in range(elt_num)]))
                array_elts = []
            array_elts.append(np.float(line.strip().replace('[', '').replace(']', '')))
    return array_list