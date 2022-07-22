import cvxpy as cp
import numpy as np
import scipy as scp
from scipy import integrate, optimize
import sympy as smp

def p_f(t_final: float) -> np.array:
    """ Generate 5th order polynomial coeffs for parameter p(t) 
        given the final time """
    t = t_final
    A = np.array([[1, 0, 0,    0,    0,    0],              # p(0) = 0
                  [1, t, t**2, t**3, t**4, t**5],           # p(t_final) = 1
                  [0, 1, 0,    0,    0,    0],              # pdot(0) = 0
                  [0, 1, 2*t,  3*t**2, 4*t**3,  5*t**4],    # pdot(t_final) = 0 
                  [0, 0, 2,    0,      0,       0],         # pdotdot(0) = 0
                  [0, 0, 2,    6*t,    12*t**2, 20*t**3]])  # pdotdot(t_final) = 0
    b = np.array([[0],
                  [1],
                  [0],
                  [0],
                  [0],
                  [0]])
    return np.linalg.solve(A, b)

global p_cfs, lower_bound, upper_bound
lower_bound = 0
upper_bound = 1
p_cfs = p_f(upper_bound).T.flatten()

def obj_func(i):
    a0 = i[0]
    a1 = i[1]
    a2 = i[2]
    a3 = i[3]
    b0 = i[4]
    b1 = i[5]
    b2 = i[6]
    b3 = i[7]
    t = smp.Symbol('t')
    p = p_cfs[0] + p_cfs[1] * t + p_cfs[2] * t**2 + p_cfs[3] * t**3 + p_cfs[4] * t**4 + p_cfs[5] * t**5
    x_p = a0 + a1*p + a2*p**2 + a3*p**3
    y_p = b0 + b1*p + b2*p**2 + b3*p**3
    f = smp.diff(x_p, t) ** 2 + smp.diff(y_p, t) ** 2
    f = smp.lambdify(t, f, 'scipy')
    integ = integrate.romberg(f, 0, 1)
    # print(str(f))
    #integral = smp.integrate(f, (t, lower_bound, upper_bound))
    return integ

def x_t(i):
    a0 = i[0]
    a1 = i[1]
    a2 = i[2]
    a3 = i[3]
    t = smp.Symbol('t')
    p = p_cfs[0] + p_cfs[1] * t + p_cfs[2] * t**2 + p_cfs[3] * t**3 + p_cfs[4] * t**4 + p_cfs[5] * t**5
    x_p = a0 + a1*p + a2*p**2 + a3*p**3

def spline_interpolation(s0, s1): 
    x_0 = s0[0]
    y_0 = s0[1]
    constraints = []
    i = optimize.minimize(obj_func, [1,1,1,1,1,1,1,1])
    print(i.x)

spline_interpolation()

#i = obj_func([1,1,1,1,500,1,1,31,4,1,4,1])
#print(str(i))