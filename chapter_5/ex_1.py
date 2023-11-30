import modern_robotics as mr
import math
import numpy as np

Slist = np.array([ [0,0,0] , [0,0,0] , [1,1,1] , [0,0,0] , [0,-1,-2] , [0,0,0] ])
thetalist = np.array([ [0] , [math.pi/4] , [0] ])
Fs = np.array([ [0] , [0] , [0] , [2] , [0] , [0] ])

Js = mr.JacobianSpace(Slist,thetalist)

print(Js)

tau = np.dot(Js.transpose(), Fs)

print(tau)
