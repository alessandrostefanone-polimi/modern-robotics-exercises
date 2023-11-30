import numpy as np
import modern_robotics as mr 
import math

T_sa = [ [0,-1,0,0] , [0,0,-1,0] , [1,0,0,1] , [0,0,0,1] ];
S_theta_se3 = mr.MatrixLog6(T_sa);
S_theta = mr.se3ToVec(S_theta_se3);
[S, theta] = mr.AxisAng6(S_theta);

print(S)
print(theta)
