import modern_robotics as mr 

S_theta = [ [0, -1.5708, 0, 2.3562], [1.5708,0,0,-2.3562], [0,0,0,1], [0,0,0,0] ];

T = mr.MatrixExp6(S_theta);
print(T);
