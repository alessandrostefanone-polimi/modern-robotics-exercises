import modern_robotics as mr
import math
import numpy as np

Blist = np.array([ [0,-1,0] , [1,0,0] , [0,0,0] , [3,0,0] , [0,3,0] , [0,0,1] ])
thetalist = np.array([ [math.pi/2] , [math.pi/2] , [1] ])

Jb = mr.JacobianBody(Blist, thetalist)

print(Jb)
