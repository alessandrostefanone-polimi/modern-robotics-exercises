# Import necessary functions and libraries
from functions import *
import matplotlib.pyplot as plt

# Define initial transformation matrices and robot configuration
T_se_in = [[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]]
T_sc_in = [[1,0,0,0.5],[0,1,0,1],[0,0,1,0.025],[0,0,0,1]]
T_sc_fin = [[1,0,0,1],[0,0,1,0.5],[0,0,1,0.025],[0,0,0,1]]
robotConfig = [0.5236,-0.2,-0.2,0,0,0.2,-1.6,0,0,0,0,0,0]

# Calculate the transformation matrix for the given robot configuration
X = calculateT(robotConfig)

# Define the proportional and integral gain matrices
Kp = np.dot(10,np.eye(6))
Ki = np.eye(6)

# Define the main function
def main(T_sc_in, T_sc_fin, robotConfig, X, T_se_in, Kp, Ki):
    # Define the grasp and standoff configurations of the end-effector in the cube frame
    T_ce_grasp = [[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]]
    T_ce_standoff = [[0,0,1,0],[0,1,0,0],[-1,0,0,0.4],[0,0,0,1]]
    
    # Define the time scaling factor
    k = 1
    
    # Generate the reference trajectory
    [refTrajectory, refTrajectoryMatrix] = TrajectoryGenerator(T_se_in, T_sc_in, T_sc_fin, T_ce_grasp, T_ce_standoff, k)
    refTrajectoryMatrix = np.array(refTrajectoryMatrix)
    
    # Initialize variables
    counter = 0
    Dt = 0.01
    maxAngSpeed = 1000
    maxArmSpeed = 1000
    error_integral = None
    X_err_matrix = []
    
    # Loop through each trajectory
    for idx_traj,traj in enumerate(refTrajectory):
        for idx in range(len(traj)-1):
            # Calculate the control input and error
            [V,X_err,error_integral] = FeedbackControl(X, traj[idx], traj[idx+1], Kp, Ki, Dt, error_integral)
            
            # Append the error to the error matrix
            X_err_matrix.append(X_err)
            
            # Calculate the Jacobian and control speeds
            Je = CalculateJacobian(robotConfig[:3],robotConfig[3:8])
            controls = CalculateSpeeds(Je, V)
            
            # Update the robot configuration
            robotConfig = NextState(robotConfig[:12], controls, Dt, maxAngSpeed, maxArmSpeed)
            robotConfig.append(int(refTrajectoryMatrix[counter,12]))
            counter += 1
            
            # Write the robot configuration and error to csv files
            with open('./steps.csv', 'a') as f:
                writer = csv.writer(f)
                writer.writerow(robotConfig)
            with open('./error.csv', 'a') as error:
                writer = csv.writer(error)
                writer.writerow(X_err)
            
            # Update the transformation matrix
            X = calculateT(robotConfig)
            
            # Print the first and last actual configurations for each trajectory
            if idx == 0:
                print("trajectory", idx_traj+1)
                print("First Actual Configuration T_se: ", X)
            if idx == 198:
                print("Last Actual Configuration T_se: ", X)
                print("-----------")
    
    # Return the error matrix
    return X_err_matrix

# Call the main function and convert the error matrix to a numpy array
X_err_matrix = np.array(main(T_sc_in,T_sc_fin,robotConfig,X,T_se_in,Kp,Ki))

# Plot the rotational and translational components of the error
fig, (ax1, ax2) = plt.subplots(2, 1)
Dt = 0.01
t = np.linspace(0,30,np.shape(X_err_matrix)[0])
ax1.plot(t, X_err_matrix[:,0],label='omega_x')
ax1.plot(t,X_err_matrix[:,1],label='omega_y')
ax1.plot(t,X_err_matrix[:,2],label='omega_z')
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('X_err (rotational) [m/s]')
ax1.legend()
ax1.grid(True)

ax2.plot(t,X_err_matrix[:,3],label='v_x')
ax2.plot(t,X_err_matrix[:,4],label='v_y')
ax2.plot(t,X_err_matrix[:,5],label='v_z')
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('X_err (translational) [m/s]')
ax2.legend()
ax2.grid(True)

# Save and display the plot
plt.tight_layout()
plt.savefig('X_err.png')
plt.show()
