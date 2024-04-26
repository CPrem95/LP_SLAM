import numpy as np
import numpy.linalg as linalg
import random

# Function is 
# f(x, y) = u0*x^3 + u1*x^2*(y^2 + u2*y)
def f(x, y, z, u):
    return u[0] * x**3 + u[1] * x**2 * (y**2 + u[2] * y) - z

def J(x, y, z, u):
    totalJ = 0
    for i in range(len(x)):
        totalJ += (f(x[i], y[i], z[i], u))**2
    return 0.5 * totalJ    

def df_du0(x, y, u):
    return x**3
    
def df_du1(x, y, u):
    return x**2 * (y**2 + u[2] * y)
    
def df_du2(x, y, u):
    return u[1] * x**2 * y

def calculateDeltaU(H_sum, J_sum):
    return linalg.lstsq(H_sum, J_sum, rcond=None)

def jacobian(x, y, z, u):
    nablaF = np.array([[df_du0(x, y, u)], [df_du1(x, y, u)], [df_du2(x, y, u)]], dtype = np.float64)
    return f(x, y, z, u) * nablaF

def hessian(x, y, u):
    nablaF = np.array([[df_du0(x, y, u)], [df_du1(x, y, u)], [df_du2(x, y, u)]], dtype = np.float64)
    return nablaF * nablaF.T

def createDataSet(x, y, u):
    z = []
    for i in range(len(x)):
        noise = random.random() * 0.01
        z.append(f(x[i], y[i], 0, u) + noise)
        #z.append(f(x[i], y[i], 0, u))
    return z

def levenbergMarquardt(x, y, z, u0, eps):
    
    c = 0.0001
    u = np.array(u0).reshape((3, 1)).astype(float)
    tmpu = u
    oldDeltaU = np.zeros((3, 1))
    deltaU = np.zeros((3, 1))
    newJ = J(x, y, z, u0)
    oldJ = newJ
    
    while(True):

        J_sum = np.zeros((3, 1))
        H_sum = np.zeros((3, 3))
    
        for i in range(len(x)):
            H_sum = H_sum + hessian(x[i], y[i], u)
            J_sum = J_sum + jacobian(x[i], y[i], z[i], u)
        
        H_sum = H_sum + c * H_sum * np.identity(3)             
        tmpDeltaU = calculateDeltaU(H_sum, J_sum)[0]
        tmpu -= tmpDeltaU    
        oldJ = newJ
        newJ = J(x, y, z, u)
        
        if (newJ > oldJ):
            c = 10 * c
            tmpu = u
            tmpDeltaU = deltaU
        else:
            c = 0.1 * c
            u = tmpu
            oldDeltaU = deltaU  
            deltaU = tmpDeltaU

        if (linalg.norm(deltaU[0] - oldDeltaU) < eps):
            break
    
    return u, newJ

if __name__ == "__main__":
    
    trueU = [0.8, 0.8, 0.8]
    u0 = [10, 10, 20]
    x = [1 for i in range(10)]
    y = [0.5 * i for i in range(10)]
    z = createDataSet(x, y, trueU)         
    
    u, Jval = levenbergMarquardt(x, y, z, u0, 0.000001)
    print("Estimated u : {0}, J : {1}".format(u, Jval))