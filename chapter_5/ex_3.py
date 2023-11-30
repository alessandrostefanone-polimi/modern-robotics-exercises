import modern_robotics as mr
import math

Slist = [ [0,1,0] , [0,0,0] , [1,0,0] , [0,0,0] , [0,2,1] , [0,0,0] ]
thetalist = [ [math.pi/2] , [math.pi/2] , [1] ]

Js = mr.JacobianSpace(Slist,thetalist)

print(Js)
