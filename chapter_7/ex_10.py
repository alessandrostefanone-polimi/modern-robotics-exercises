import modern_robotics as mr
import numpy as np

T_sb = np.array([ [1,0,0,0], [0,0,1,2], [0,-1,0,0], [0,0,0,1] ]);
F_b = np.array([1,0,0,2,1,0]);
F_b = F_b.reshape(-1,1);
print(F_b);
AdT_sb = mr.Adjoint(T_sb);
print(AdT_sb);
F_s = np.dot(AdT_sb, F_b);
print(F_s)
