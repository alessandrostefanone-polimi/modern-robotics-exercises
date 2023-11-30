import modern_robotics as mr
import numpy as np

S_theta = [0,1,2,3,0,0];
S_theta_se3 = mr.VecTose3(S_theta);
T = mr.MatrixExp6(S_theta_se3);

print(T);
