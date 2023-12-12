import numpy as np

def null_project(robot, q, qnull, ev, λ):
    """
    Calculates the required joint velocities qd to achieve the desired
    end-effector velocity ev while projecting the null-space motion qnull
    into the null-space of the jacobian of the robot.

    robot: a Robot object (must be redundant with robot.n > 6)
    q: the robots current joint coordinates
    qnull: the null-space motion to be projected into the solution
    ev: the desired end-effector velocity (expressed in the base-frame
        of the robot)
    λ: a gain to apply to the null-space motion

    Note: If you would like to express ev in the end-effector frame,
        change the `jacob0` below to `jacobe`
    """

    # Calculate the base-frame manipulator Jacobian
    J0 = robot.jacob0(q)

    # Calculate the pseudoinverse of the base-frame manipulator Jacobian
    J0_pinv = np.linalg.pinv(J0)

    # Calculate the joint velocity vector according to the equation above
    qd = J0_pinv @ ev + (1.0 / λ) * (np.eye(robot.n) - J0_pinv @ J0) @ qnull.reshape(robot.n,)

    return qd

def jacobm(robot, q, axes):
    """
    Calculates the manipulability Jacobian. This measure relates the rate
    of change of the manipulability to the joint velocities of the robot.

    q: The joint angles/configuration of the robot
    axes: A boolean list which correspond with the Cartesian axes to
        find the manipulability Jacobian of (6 boolean values in a list)

    returns the manipulability Jacobian
    """

    # Calculate the base-frame manipulator Jacobian
    J0 = robot.jacob0(q)

    # only keep the selected axes of J
    J0 = J0[axes, :]

    # Calculate the base-frame manipulator Hessian
    H0 = robot.hessian0(q)

    # only keep the selected axes of H
    H0 = H0[:, axes, :]

    # Calculate the manipulability of the robot
    manipulability = np.sqrt(np.linalg.det(J0 @ J0.T))

    # Calculate component of the Jacobian
    b = np.linalg.inv(J0 @ J0.T)

    # Allocate manipulability Jacobian
    Jm = np.zeros((robot.n, 1))

    # Calculate manipulability Jacobian
    for i in range(robot.n):
        c = J0 @ H0[i, :, :].T
        Jm[i, 0] = manipulability * (c.flatten("F")).T @ b.flatten("F")

    return Jm