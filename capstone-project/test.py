import numpy as np

Xd = [[0,0,1,0.5],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]]
Xd_next = [[0,0,1,0.6],[0,1,0,0],[-1,0,0,0.3],[0,0,0,1]]
X = [[0.170,0,0.985,0.387],[0,1,0,0],[-0.985,0,0.170,0.570],[0,0,0,1]]
Dt = 0.01
error_integral = np.zeros((4,4))
Ki = np.zeros((4,4))
X_err = np.log(np.dot(np.linalg.inv(X),Xd)+1e-10)
error_integral = error_integral + np.dot(X_err,Dt)

print(np.dot(Ki,error_integral))