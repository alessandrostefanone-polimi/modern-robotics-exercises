def ForwardDynamicsTrajectorySteps(thetalist, dthetalist, taumat, g, Ftipmat, \
                              Mlist, Glist, Slist, dt, intRes):
    """Simulates the motion of a serial chain given an open-loop history of
    joint forces/torques

    :param thetalist: n-vector of initial joint variables
    :param dthetalist: n-vector of initial joint rates
    :param taumat: An N x n matrix of joint forces/torques, where each row is
                   the joint effort at any time step
    :param g: Gravity vector g
    :param Ftipmat: An N x 6 matrix of spatial forces applied by the end-
                    effector (If there are no tip forces the user should
                    input a zero and a zero matrix will be used)
    :param Mlist: List of link frames {i} relative to {i-1} at the home
                  position
    :param Glist: Spatial inertia matrices Gi of the links
    :param Slist: Screw axes Si of the joints in a space frame, in the format
                  of a matrix with axes as the columns
    :param dt: The timestep between consecutive joint forces/torques
    :param intRes: Integration resolution is the number of times integration
                   (Euler) takes places between each time step. Must be an
                   integer value greater than or equal to 1
    :return thetamat: The N x n matrix of robot joint angles resulting from
                      the specified joint forces/torques
    :return dthetamat: The N x n matrix of robot joint velocities
    This function calls a numerical integration procedure that uses
    ForwardDynamics.

    """

    taumat = np.array(taumat).T
    Ftipmat = np.array(Ftipmat).T
    #initialize a matrix to keep track of the joint angles during each step of integration
    thetamat_step = np.zeros((6,intRes))
    #create an empty vector to populate with the values from the iterations
    thetamat = np.array([[], [], [], [], [], []])
    #initialize a matrix to keep track of the joint velocities during each step of integration
    dthetamat_step = np.zeros((6,intRes))
    #create an empty vector to populate with the values from the iterations
    dthetamat = np.array([[], [], [], [], [], []])

    for i in range(np.array(taumat).shape[1]):
        for j in range(intRes):
            ddthetalist \
            = ForwardDynamics(thetalist, dthetalist, taumat[:, i], g, \
                              Ftipmat[:, i], Mlist, Glist, Slist)
            thetalist,dthetalist = EulerStep(thetalist, dthetalist, \
                                             ddthetalist, 1.0 * dt / intRes)
            #assemble a matrix with 6 rows and len(intRes) columns, each column contains the current configuration in terms of joint angles and joint velocities
            thetamat_step[:, j] = thetalist
            dthetamat_step[:, j] = dthetalist
        #assemble the total matrix stacking horizontally the matrix with the values at each step of integration for the i-th iteration
        thetamat = np.hstack((thetamat, thetamat_step))
        dthetamat = np.hstack((dthetamat, dthetamat_step))
    thetamat = np.array(thetamat).T
    dthetamat = np.array(dthetamat).T
    return thetamat, dthetamat
