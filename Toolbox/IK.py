import numpy as np
from collections import namedtuple

import roboticstoolbox as rtb
from spatialmath import base
from spatialmath import SE3
import math
from typing import Union, Tuple
from collections import namedtuple

iksol = namedtuple("IKsolution", "q, success, reason, iterations, residual")

def _angle_axis(T, Td):
    d = base.transl(Td) - base.transl(T)
    R = base.t2r(Td) @ base.t2r(T).T
    li = np.r_[R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]

    if base.iszerovec(li):
        # diagonal matrix case
        if np.trace(R) > 0:
            # (1,1,1) case
            a = np.zeros((3,))
        else:
            a = np.pi / 2 * (np.diag(R) + 1)
    else:
        # non-diagonal matrix case
        ln = base.norm(li)
        a = math.atan2(ln, np.trace(R) - 1) * li / ln

    return np.r_[d, a]

class IK():
    def ikine_LMS(
        self, T: SE3, q0=None, mask=None, ilimit=500, tol=1e-10, wN=1e-3, Lmin=0, end=None
    ):
        """
        Numerical inverse kinematics by Levenberg-Marquadt optimization
        (Robot superclass)

        :param T: The desired end-effector pose or pose trajectory
        :type T: SE3
        :param q0: initial joint configuration (default all zeros)
        :type q0: ndarray(n)
        :param mask: mask vector that correspond to translation in X, Y and Z
            and rotation about X, Y and Z respectively.
        :type mask: ndarray(6)
        :param ilimit: maximum number of iterations (default 500)
        :type ilimit: int
        :param tol: final error tolerance (default 1e-10)
        :type tol: float
        :param ωN: damping coefficient
        :type ωN: float (default 1e-3)
        :return: inverse kinematic solution
        :rtype: named tuple

        ``sol = robot.ikine_LM(T)`` are the joint coordinates (n) corresponding
        to the robot end-effector pose ``T`` which is an ``SE3`` object. This
        method can be used for robots with any number of degrees of freedom.
        The return value ``sol`` is a named tuple with elements:

        ============    ==========  ===============================================
        Element         Type        Description
        ============    ==========  ===============================================
        ``q``           ndarray(n)  joint coordinates in units of radians or metres
        ``success``     bool        whether a solution was found
        ``reason``      str         reason for the failure
        ``iterations``  int         number of iterations
        ``residual``    float       final value of cost function
        ============    ==========  ===============================================

        If ``success=False`` the ``q`` values will be valid numbers, but the
        solution will be in error.  The amount of error is indicated by
        the ``residual``.

        **Trajectory operation**:

        If ``len(T) = m > 1`` it is considered to be a trajectory, then the
        result is a named tuple whose elements are

        ============    ============   ===============================================
        Element         Type           Description
        ============    ============   ===============================================
        ``q``           ndarray(m,n)   joint coordinates in units of radians or metres
        ``success``     bool(m)        whether a solution was found
        ``reason``      list of str    reason for the failure
        ``iterations``  ndarray(m)     number of iterations
        ``residual``    ndarray(m)     final value of cost function
        ============    ============   ===============================================

        **Underactuated robots:**

        For the case where the manipulator has fewer than 6 DOF the
        solution space has more dimensions than can be spanned by the
        manipulator joint coordinates.

        In this case we specify the ``mask`` option where the ``mask`` vector
        (6) specifies the Cartesian DOF (in the wrist coordinate frame) that
        will be ignored in reaching a solution.  The mask vector has six
        elements that correspond to translation in X, Y and Z, and rotation
        about X, Y and Z respectively. The value should be 0 (for ignore)
        or 1. The number of non-zero elements should equal the number of
        manipulator DOF.

        For example when using a 3 DOF manipulator rotation orientation might
        be unimportant in which case use the option: mask = [1 1 1 0 0 0].

        .. note::

            - See `Toolbox kinematics wiki page
                <https://github.com/petercorke/robotics-toolbox-python/wiki/Kinematics>`_
            - Implements a modified Levenberg-Marquadt variable-damping solver
                which is quite robust in practice.
            - Similar to ``ikine_LM`` but uses a different error metric
            - The tolerance is computed on the norm of the error between
                current and desired tool pose.  This norm is computed from
                distances and angles without any kind of weighting.
            - The inverse kinematic solution is generally not unique, and
                depends on the initial guess ``q0``.
            - The default value of ``q0`` is zero which is a poor choice for
                most manipulators since it often corresponds to a
                kinematic singularity.
            - Such a solution is completely general, though much less
                efficient than analytic inverse kinematic solutions derived
                symbolically.
            - This approach allows a solution to be obtained at a singularity,
                but the joint angles within the null space are arbitrarily
                assigned.
            - Joint offsets, if defined, are accounted for in the solution.
            - Joint limits are not considered in this solution.

        :references:
            - "Solvability-Unconcerned Inverse Kinematics by the
                Levenberg–Marquardt Method", T. Sugihara, IEEE T-RO, 27(5),
                October 2011, pp. 984-991.

        :seealso: :func:`ikine_LM`, :func:`ikine_unc`, :func:`ikine_con`,
            :func:`ikine_min`
        """  # noqa E501

        if not isinstance(T, SE3):
            raise TypeError("argument must be SE3")

        if isinstance(self, rtb.DHRobot):
            end = None

        solutions = []

        if q0 is None:
            q0 = np.zeros((self.n,))
        else:
            q0 = base.getvector(q0, self.n)

        if mask is not None:
            mask = base.getvector(mask, 6)
            if not self.n >= np.sum(mask):
                raise ValueError(
                    "Number of robot DOF must be >= the number "
                    "of 1s in the mask matrix"
                )
        else:
            mask = np.ones(6)
        W = np.diag(mask)

        tcount = 0  # Total iteration count
        revolutes = self.revolutejoints

        q = q0
        for Tk in T:
            iterations = 0
            failure = None
            while True:
                # Update the count and test against iteration limit
                iterations += 1

                if iterations > ilimit:
                    failure = f"iteration limit {ilimit} exceeded"
                    break

                e = _angle_axis(self.fkine(q, end=end).A, Tk.A)

                # Are we there yet?
                E = 0.5 * e.T @ W @ e
                if E < tol:
                    break

                # Compute the Jacobian and projection matrices
                J = self.jacob0(q, end=end)
                WN = E * np.eye(self.n) + wN * np.eye(self.n)
                H = J.T @ W @ J + WN  # n x n
                g = J.T @ W @ e  # n x 1

                # Compute new value of q
                q += np.linalg.inv(H) @ g  # n x 1
                # print(np.linalg.norm(np.linalg.inv(H) @ g))
                # print(e)
                # print(g)
                # print(q)
                # print(J)

                # Wrap angles for revolute joints
                k = np.logical_and(q > np.pi, revolutes)
                q[k] -= 2 * np.pi

                k = np.logical_and(q < -np.pi, revolutes)
                q[k] += +2 * np.pi

                # qs = ", ".join(["{:8.3f}".format(qi) for qi in q])
                # print(f"|e|={E:8.2g}, det(H)={np.linalg.det(H)}: q={qs}")

            # LM process finished, for better or worse
            # failure will be None or an error message
            solution = iksol(q, failure is None, failure, iterations, E)
            solutions.append(solution)

            tcount += iterations

        if len(T) == 1:
            return solutions[0]
        else:
            return iksol(
                np.vstack([sol.q for sol in solutions]),
                np.array([sol.success for sol in solutions]),
                [sol.reason for sol in solutions],
                np.array([sol.iterations for sol in solutions]),
                np.array([sol.residual for sol in solutions]),
            )