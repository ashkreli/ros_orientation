import cvxpy as cp
import numpy as np
import math
from typing import Tuple, List
import matplotlib.pyplot as plt
import yaml
import sim_move, lqr

global N, I
N = 50
I = 2
dt = 1 / I

def objective_function(pts: List[Tuple[float, np.array]]):
    def make_A_t(point: Tuple[float, np.array]):
        t = point[0]
        A_t = np.array([[1, t, t**2, t**3, 0, 0, 0, 0],
                        [0, 0, 0, 0, 1, t, t**2, t**3]])
        return A_t
    A = make_A_t(pts[0])
    b = pts[0][1]
    for i in range(1, len(pts)):
        A = np.concatenate((A, make_A_t(pts[i])), axis=0)
        b = np.concatenate((b, pts[i][1]), axis=0)
    return A, b

def find_a_b(pts: List[Tuple[float, np.array]]):
    a_b = cp.Variable((8, 1))
    A, b = objective_function(pts)
    dist = cp.sum_squares(A @ a_b - b)
    prob = cp.Problem(cp.Minimize(dist))
    prob.solve()

    return a_b.value

def atan2(y, x):
    a = []
    for i in range(len(y)):
        a.append(math.atan2(y[i], x[i]))
    return a

def sqrt(lst):
    a = []
    for el in lst:
        a.append(math.sqrt(el))
    return a

def gen_s_u(waypts):
    a_b = find_a_b(waypts)
    t = np.linspace(waypts[0][0], waypts[-1][0])
    a = a_b[0:4]
    b = a_b[4:]
    x = a[0] + a[1] * t + a[2] * t**2 + a[3] * t**3
    print(str(type(x)))
    y = b[0] + b[1] * t + b[2] * t**2 + b[3] * t**3
    x_dot = a[1] + 2 * a[2] * t + 3 * a[3] * t**2
    y_dot = b[1] + 2 * b[2] * t + 3 * b[3] * t**2
    theta = atan2(y_dot, x_dot)
    v = sqrt(x_dot**2 + y_dot**2)
    x_dotdot = 2 * a[2] + 6 * a[3] * t
    y_dotdot = 2 * b[2] + 6 * b[3] * t
    omega = -x_dotdot * y_dot / v + y_dotdot * x_dot / v
    s_t = []
    u_t = []
    for i in range(len(x)):
        s_t.append(np.array([[x[i]], [y[i]], [theta[i]]]))
        u_t.append(np.array([[v[i]], [omega[i]]]))
    return s_t, u_t


waypts = [(0, np.array([[1], [1]])), 
          (1, np.array([[2], [4]])), 
          (2, np.array([[5], [2]]))]

def unpack(s):
    x = []
    y = []
    for item in s:
        x.append(item.T[0][0])
        y.append(item.T[0][1])
    return(np.array(x), np.array(y))

with open('start.yaml') as f:
    start = yaml.safe_load(f)
    x_0 = start['x_0']
    y_0 = start['y_0']
    theta_0 = start['theta_0']
    x_line_0 = start['x_line_0']
    y_line_0 = start['y_line_0']
    x_line_N = start['x_line_N']
    y_line_N = start['y_line_N']

global s_0
s_0 = np.array([[x_0],[y_0],[theta_0]])

s_refs, u_refs = gen_s_u(waypts)
# print(s_t)

s = [s_0]

for i in range(N):
    for j in range(I):
        # get the next control input vector from the lqr controller
        u = lqr.lqr_traj_track(s[-1], s_refs[i], u_refs[i], dt)
        # get the system's state and control input matrices
        A = sim_move.getA()
        B = sim_move.getB((s[-1].T)[0][2], dt)
        # print("Theta " + str(i) + ":" + str((s[-1].T)[0][2]))
        # calculate the next step according to the state space model
        s.append(sim_move.state_space_model(A, s[-1], B, u))
plt.title("shape") 
plt.xlabel("x") 
plt.ylabel("y")
(x_ref, y_ref) = unpack(s_refs)
(x, y) = unpack(s)
plt.plot(x, y, 'bo') 
plt.plot(x_ref, y_ref, 'go')
plt.show()