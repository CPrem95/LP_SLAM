import numpy as np
from scipy.optimize import least_squares

rob_x = 5000
rob_y = 1000

anc_x = 500
anc_y = 3500

rob_r = 200

# generate the circle points
def circle(rob_x, rob_y, rob_r, n):
    N = n + 1
    th = np.linspace(0, 2*np.pi, N)
    x_r = rob_x + rob_r * np.cos(th)
    y_r = rob_y + rob_r * np.sin(th)
    return x_r, y_r

# find the distance between the robot and the anchor
def find_L(x_r, y_r, x_a, y_a, alp):
    L = np.sqrt((x_r - x_a)**2 + (y_r - y_a)**2)
    L = L + np.random.normal(-200, 200, len(L))
    return L

def fun_objective(x, xdata, L):
    return (x[0] - xdata[0][:])**2 + (x[1] - xdata[1][:])**2 - L**2

######################################################################
# main

# generate the circle points
x_r, y_r = circle(rob_x, rob_y, rob_r, 100)

# find the distance between the robot and the anchor
L = find_L(x_r, y_r, anc_x, anc_y, 0)

# initial guess
# x0_fun = np.array([500, -2000])
x0_fun = np.array([5000, 1000])

# least square optimization
res_1 = least_squares(fun_objective, x0_fun, args=([x_r, y_r], L), method='lm')

# print the result
def main():
    print('Hi from uwb_slam.')
    print('res_1:', res_1)

if __name__ == '__main__':
    main()
