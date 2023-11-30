import modern_robotics as mr

Xstart = [ [1,0,0,0] , [0,1,0,0] , [0,0,1,0] , [0,0,0,1] ]
Xend = [ [0,0,1,1], [1,0,0,2], [0,1,0,3], [0,0,0,1] ]
Tf = 10
N = 10
method = 5

traj = mr.CartesianTrajectory(Xstart, Xend, Tf, N, method)

print(traj[8])
