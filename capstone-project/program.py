from functions import *

T_se_in = [[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]]
T_sc_in = [[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]]
T_sc_fin = [[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]]
robotConfig = [0,0,0,0,0,0.2,-1.6,0,0,0,0,0,0]
X = [[0.170,0,0.985,0.387],[0,1,0,0],[-0.985,0,0.170,0.570],[0,0,0,1]]
Kp = np.zeros(6)
Ki = np.zeros(6)
def main(T_sc_in, T_sc_fin, robotConfig, X, T_se_in, Kp, Ki):
    T_ce_grasp = [[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]]
    T_ce_standoff = [[0,0,1,0],[0,1,0,0],[-1,0,0,0.4],[0,0,0,1]]
    k = 1
    [refTrajectory, refTrajectoryMatrix] = TrajectoryGenerator(T_se_in, T_sc_in, T_sc_fin, T_ce_grasp, T_ce_standoff, k)
    refTrajectoryMatrix = np.array(refTrajectoryMatrix)
    counter = 0
    Dt = 0.01
    maxAngSpeed = 10
    maxArmSpeed = 10
    error_integral = None
    X_err_matrix = []
    for idx_traj,traj in enumerate(refTrajectory):
        for idx in range(len(traj)-1):
            print("trajectory", idx_traj+1)
            print("step: ", idx)
            [V,X_err,error_integral] = FeedbackControl(X, traj[idx], traj[idx+1], Kp, Ki, Dt, error_integral) #X should be updated at each cycle
            print("X_err vector: ", X_err)
            X_err_matrix.append(X_err)
            Je = CalculateJacobian(robotConfig[:3],robotConfig[3:8])
            controls = CalculateSpeeds(Je, V)
            robotConfig = NextState(robotConfig[:12], controls, Dt, maxAngSpeed, maxArmSpeed)
            robotConfig.append(int(refTrajectoryMatrix[counter,12]))
            counter += 1
            #print("counter: ", counter)
            print("-----------")
            with open('./steps.csv', 'a') as f:
                writer = csv.writer(f)
                writer.writerow(robotConfig)
    return X_err_matrix

X_err_matrix = main(T_sc_in,T_sc_fin,robotConfig,X,T_se_in,Kp,Ki)