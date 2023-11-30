import math as m
import numpy as np
import csv
import modern_robotics as mr
from scipy.linalg import logm,expm

def calculate_geometry(phi,x,y):
    T_sb = [[m.cos(phi),-m.sin(phi),0,x], [m.sin(phi),m.cos(phi),0,y],[0,0,1,0.0963],[0,0,0,1]]
    T_b0 = [[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]]
    M_0e = [[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]]
    Blist = np.array([[0,0,0,0,0],[0,-1,-1,-1,0],[1,0,0,0,1],[0,-0.5076,-0.3526,-0.2176,0],[0.033,0,0,0,0],[0,0,0,0,0]])
    return T_sb, T_b0, M_0e, Blist

def NextState(currentConfig, velocities, timestep, maxAngularSpeedWheel, maxAngularSpeedArm):
    for idx,velocity in enumerate(velocities[0:4]):
        if velocity >= 0:
            if velocity > maxAngularSpeedWheel:
                velocities[idx] = maxAngularSpeedWheel
        elif velocity < -maxAngularSpeedWheel:
            velocities[idx] = -maxAngularSpeedWheel

    for idx,velocity in enumerate(velocities[4:len(velocities)]):
        if velocity >= 0:
            if velocity > maxAngularSpeedArm:
                velocities[idx] = maxAngularSpeedArm
        elif velocity < -maxAngularSpeedArm:
            velocities[idx] = -maxAngularSpeedArm

    r = 0.0475
    w = 0.3/2
    l = 0.47/2
    F = [[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1,1,1,1], [-1,1,-1,1]]
    F = np.dot(r/4,F)
    print(F)
    deltaTheta = [[velocities[0]*timestep],[velocities[1]*timestep],[velocities[2]*timestep],[velocities[3]*timestep]]
    print(deltaTheta)
    Vb = np.dot(F,deltaTheta).flatten()
    print(Vb)
    Vb6 = [0,0,Vb[0],Vb[1],Vb[2],0]
    if abs(Vb[0]) < 1e-6:
        deltaQb = [0,Vb[1],Vb[2]]
    else:
        deltaQb = [Vb[0],(Vb[1]*m.sin(Vb[0]) + Vb[2]*(m.cos(Vb[0])-1))/Vb[0],(Vb[2]*m.sin(Vb[0]) + Vb[1]*(1-m.cos(Vb[0])))/Vb[0]]
    print(deltaQb)
    transfMat = [[1,0,0],
                 [0,m.cos(currentConfig[0]),-m.sin(currentConfig[0])],
                 [0,m.sin(currentConfig[0]),m.cos(currentConfig[0])]]
    deltaQ = np.dot(transfMat, deltaQb).flatten()
    currentConfig[0:3] = currentConfig[0:3] + deltaQ
    currentConfig[3:8] = currentConfig[3:8] + np.array(velocities[4:])*timestep
    currentConfig[8:] = currentConfig[8:] + np.array(velocities[0:4])*timestep

    return currentConfig

def testNextState():
    timestep = 0.01
    currentConfig = [0, 0, 0, 0.80405, -0.91639, -0.011436, 0.054333, 0.00535, 0, 0, 0, 0]
    velocities = [-10,10,10,-10,0,0,0,0,0]
    maxAngSpeed = 5
    maxArmSpeed = 5
    i = 0
    while i < 100:
        currentConfig = NextState(currentConfig,velocities,timestep,maxAngSpeed,maxArmSpeed)
        vector = np.concatenate([currentConfig,[0]])
        with open('./steps.csv', 'a') as f:
            writer = csv.writer(f)
            writer.writerow(vector)
        i+=1

def TrajectoryGenerator(T_se_in,T_sc_in,T_sc_fin,T_ce_grasp,T_ce_standoff,k):
    Tf = 2
    T_se_standoff = np.dot(T_sc_in,T_ce_standoff)
    traj1 = mr.ScrewTrajectory(T_se_in,T_se_standoff,Tf,Tf*k/0.01,5)
    T_se_grasp = np.dot(T_sc_in,T_ce_grasp)
    traj2 = mr.ScrewTrajectory(traj1[-1],T_se_grasp,Tf,Tf*k/0.01,5)
    traj3 = []
    for i in range(100*k):
        traj3.append(traj2[-1])
    traj4 = mr.ScrewTrajectory(traj2[-1],T_se_standoff,Tf,Tf*k/0.01,5)
    T_se_standoff_fin = np.dot(T_sc_fin,T_ce_standoff)
    traj5 = mr.ScrewTrajectory(traj4[-1],T_se_standoff_fin,Tf,Tf*k/0.01,5)
    T_se_grasp_fin = np.dot(T_sc_fin, T_ce_grasp)
    traj6 = mr.ScrewTrajectory(traj5[-1],T_se_grasp_fin,Tf,Tf*k/0.01,5)
    traj7 = []
    for i in range(100*k):
        traj7.append(traj6[-1])
    traj8 = mr.ScrewTrajectory(traj6[-1],T_se_standoff_fin,Tf,Tf*k/0.01,5)
    trajectories = [traj1,traj2,traj3,traj4,traj5,traj6,traj7,traj8]
    for traj in trajectories:
        for T in traj:
            r11 = T[0][0]
            r12 = T[0][1]
            r13 = T[0][2]
            px = T[0][3]
            r21 = T[1][0]
            r22 = T[1][1]
            r23 = T[1][2]
            py = T[1][3]
            r31 = T[2][0]
            r32 = T[2][1]
            r33 = T[2][2]
            pz = T[2][3]

            if np.array_equal(traj, traj1) or np.array_equal(traj, traj2) or np.array_equal(traj, traj7) or np.array_equal(traj, traj8):
                gripper_status = 0
            else:
                gripper_status = 1

            row_trajectory = [r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,gripper_status]

            with open('./trajectories.csv', 'a') as f:
                writer = csv.writer(f)
                writer.writerow(row_trajectory)

def testTrajectoryGeometry():
    T_sc_in = [[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]]
    T_sc_fin = [[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]]
    T_se_in = [[1,0,0,0],[0,1,0,0],[0,0,1,1],[0,0,0,1]]
    T_ce_grasp = [[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]]
    T_ce_standoff = [[0,0,1,0],[0,1,0,0],[-1,0,0,0.4],[0,0,0,1]]
    k = 1
    TrajectoryGenerator(T_se_in,T_sc_in,T_sc_fin,T_ce_grasp,T_ce_standoff,k)

def FeedbackControl(X,Xd,Xd_next,Kp,Ki,Dt,error_integral):
    Vd = []
    if not error_integral:
        error_integral = np.zeros(6)
    X_err = logm(np.dot(np.linalg.inv(X),Xd))
    X_err_vec = mr.se3ToVec(X_err)
    error_integral = error_integral + np.dot(X_err_vec,Dt)
    Vd_b = np.dot((1/Dt),logm(np.dot(np.linalg.inv(Xd),Xd_next)+ 1e-10))
    Vd = mr.se3ToVec(Vd_b)
    V = np.dot(mr.Adjoint(np.dot(np.linalg.inv(X),Xd)),Vd) + np.dot(Kp,X_err_vec) + np.dot(Ki,error_integral)
    return V


def CalculateJacobian(posConfig,armConfig):
    T_sb, T_b0, M_0e, Blist = calculate_geometry(posConfig[0],posConfig[1],posConfig[2])
    T = np.eye(4)
    Jbn = Blist[:,-1]
    Jbn = np.reshape(Jbn,(6,1))
    Jb_arm = Jbn
    T = expm(-mr.VecTose3(Blist[:,-1])*armConfig[-1])
    Jbi = np.dot(mr.Adjoint(T),Blist[:,-2])
    Jbi = np.reshape(Jbi,(6,1))
    Jb_arm = np.hstack((Jbi,Jb_arm))            

    for i in range(np.shape(Blist)[1]-2,0,-1):
        T = expm(-mr.VecTose3(Blist[:,-1])*armConfig[-1])
        for ii in range(np.shape(Blist)[1]-2,i-1,-1):
            T = np.dot(T,expm(-mr.VecTose3(Blist[:,ii])*armConfig[ii]))
        Jbi = np.dot(mr.Adjoint(T),Blist[:,i-1])
        Jbi = np.reshape(Jbi,(6,1))
        Jb_arm = np.hstack((Jbi,Jb_arm))
    T_ee_config = M_0e
    for jj in range(np.shape(Blist)[1]):
        T_ee_config = np.dot(T_ee_config,expm(mr.VecTose3(Blist[:,jj])*armConfig[jj]))
    T_0e_config = np.dot(M_0e,T_ee_config)
    T_base = np.dot(np.linalg.inv(T_0e_config),np.linalg.inv(T_b0))
    r = 0.0475
    w = 0.3/2
    l = 0.47/2
    F = [[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1,1,1,1], [-1,1,-1,1]]
    F = np.array(np.dot(r/4,F))
    m = np.shape(F)[1]
    F_six = [[np.zeros(m)],
             [np.zeros(m)],
             [F[0]],
             [F[1]],
             [F[2]],
             [np.zeros(m)]
             ]
    F_six = np.array(F_six)
    F_six = np.reshape(F_six, (6,4))
    Jb_base = np.dot(mr.Adjoint(T_base),F_six)
    Jb = np.hstack((Jb_base,Jb_arm))
    return Jb
    
def CalculateSpeeds(Je,V):
    return np.dot(np.linalg.pinv(Je),V)

def testControlandJacobian():
    robotConfig = [0,0,0,0,0,0.2,-1.6,0]
    Xd = [[0,0,1,0.5],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]]
    Xd_next = [[0,0,1,0.6],[0,1,0,0],[-1,0,0,0.3],[0,0,0,1]]
    X = [[0.170,0,0.985,0.387],[0,1,0,0],[-0.985,0,0.170,0.570],[0,0,0,1]]
    Kp = np.zeros((6,6))
    Ki = np.zeros((6,6))
    Dt = 0.01
    V = FeedbackControl(X,Xd,Xd_next,Kp,Ki,Dt,error_integral=None)
    #print(V)
    Je = np.array(CalculateJacobian(robotConfig[:3],robotConfig[3:]))
    #print(Je)
    speeds = CalculateSpeeds(Je,V)
    print(np.shape(speeds))

testControlandJacobian()





