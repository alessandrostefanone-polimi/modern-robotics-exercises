import numpy as np
import modern_robotics as mr

T_sa = np.array([ [0,-1,0,0] , [0,0,-1,0] , [1,0,0,1] , [0,0,0,1] ]);
MU_s = [ [3],[2],[1],[-1],[-2],[-3] ];
T_as = mr.TransInv(T_sa);
AdT_as = mr.Adjoint(T_as);

MU_a = np.dot(AdT_as, MU_s);
print(MU_a);
