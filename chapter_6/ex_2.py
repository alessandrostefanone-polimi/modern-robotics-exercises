import modern_robotics as mr
import math

M = [ [1,0,0,3] , [0,1,0,0] , [0,0,1,0] , [0,0,0,1] ]
Slist = [ [0,0,0] , [0,0,0] , [1,1,1] , [0,0,0] , [-2,-1, 0] , [0,0,0] ]
T = [ [-0.585, -0.811, 0, 0.076] , [0.811, -0.585, 0, 2.608] , [0,0,1,0] , [0,0,0,1] ]
Blist = [ [0,0,0] , [0,0,0] , [1,1,1] , [0,0,0] , [3,2,1] , [0,0,0] ]
thetalist0 = [ 0.7854 , 0.7854 , 0.7854 ]
eomg = 0.001
ev = 0.0001

#[thetalist, success] = mr.IKinSpace(Slist,M,T,thetalist0,eomg,ev)
[thetalist2, success2] = mr.IKinBodyIterates(Blist,M,T,thetalist0,eomg,ev)

#print(success)
print(success2)
#print(thetalist)
print(thetalist2)
