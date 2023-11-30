import modern_robotics as mr
import math

M = [ [1,0,0,3.73] , [0,1,0,0] , [0,0,1,2.73], [0,0,0,1] ];
Slist = [ [0,0,0,0,0,0] , [0,1,1,1,0,0] , [1,0,0,0,0,1] , [0,0,1,-0.73,0,0] , [-1,0,0,0,0,-3.73] , [0,1,2.73,3.73,1,0] ];
thetalist = [ [-math.pi/2], [math.pi/2], [math.pi/3], [-math.pi/4], 1, [math.pi/6] ];

T = mr.FKinSpace(M,Slist,thetalist);
print(T);

