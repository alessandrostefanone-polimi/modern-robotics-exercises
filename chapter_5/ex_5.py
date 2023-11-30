import numpy as np
import math

Jv = np.array([ [-0.105, 0, 0.006, -0.045, 0, 0.006, 0] , [-0.889, 0.006, 0, -0.844, 0.006, 0, 0] , [0, -0.105, 0.889, 0, 0, 0, 0] ])

A = np.dot(Jv,Jv.transpose())

w,v = np.linalg.eig(A)

length = math.sqrt(w[1])

print(length)

