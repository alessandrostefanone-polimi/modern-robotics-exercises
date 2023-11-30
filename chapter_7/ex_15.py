import modern_robotics as mr

T = [ [0,-1,0,3] , [1,0,0,0], [0,0,1,1], [0,0,0,1] ];

S_theta = mr.MatrixLog6(T);
print(S_theta);
