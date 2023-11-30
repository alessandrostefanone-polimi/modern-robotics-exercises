import modern_robotics as mr

s = [1,0,0];
p = [0,0,2];
h = 1; 

S = mr.ScrewToAxis(p,s,h);
print(S)
