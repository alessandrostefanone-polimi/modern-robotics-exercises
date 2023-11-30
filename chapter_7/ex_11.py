import modern_robotics as mr
import numpy as np

T = [ [0,-1,0,3], [1,0,0,0], [0,0,1,1], [0,0,0,1] ];

T_inv = mr.TransInv(T);

print(T_inv);
