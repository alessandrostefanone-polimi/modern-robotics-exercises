import modern_robotics as mr
import math

W1 = 0.109
W2 = 0.082
L1 = 0.425
L2 = 0.392
H1 = 0.089
H2 = 0.095

M = [ [-1,0,0,L1+L2] , [0,0,1,W1+W2] , [0,1,0,H1-H2] , [0,0,0,1] ]
Blist = [ [0,0,0,0,0,0] , [1,0,0,0,-1,0] , [0,1,1,1,0,1] , [W1+W2,H2,H2,H2,-W2,0] , [0,-L1-L2,-L2,0,0,0] , [L1+L2,0,0,0,0,0] ]
Slist = [ [0,0,0,0,0,0] , [0,1,1,1,0,1] , [1,0,0,0,-1,0] , [0,-H1,-H1,-H1,-W1,H2-H1] , [0,0,0,0, L1+L2, 0] , [0,0,L1,L1+L2,0,L1+L2] ]
T = [ [0,1,0,-0.5] , [0,0,-1,0.1] , [-1,0,0,0.1] , [0,0,0,1] ]
eomg = 0.001
ev = 0.0001
thetalist0 = [ 6.000, -2.500, 4.500, -5.000, 3.500, 1.500 ]

[thetalist, success] = mr.IKinBodyIterates(Blist,M,T,thetalist0,eomg,ev)

print(success)
print(thetalist)
