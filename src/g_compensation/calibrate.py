import itertools

import numpy as np
import PyKDL as kdl
import scipy.optimize


def gravity_measurement(m, com, orientation, g=(0, 0, -9.81)):
    frame = kdl.Frame(kdl.Rotation.RPY(*orientation), kdl.Vector(*com))
    force = m*kdl.Wrench(kdl.Vector(*g), kdl.Vector())
    m = frame * force
    return tuple(m.force) + tuple(m.torque)


def measurement_matrix(theta, orientations, g=(0, 0, -9.81)):
    """
    Args:
        theta: 1-D array concatenation of
            * mass
            * center of mass x, y and z
        orientations: iterable of N roll-pitch-yaw tuples
    Returns:
        an Nx6 matrix of wrenches caused by gravity in every orientation.
    """
    os = list(orientations)
    entries = itertools.chain.from_iterable(
        gravity_measurement(theta[0], theta[1:4], o, g) for o in os)
    return np.reshape(list(entries), (len(os), 6), order='C')


def matrix_error(m1, m2):
    # TODO: something better than L2 norm on wrench?
    return sum(np.apply_along_axis(np.linalg.norm, 1, m1 - m2))


def estimate(orientations, wrenches, initial_theta, tolerance=1e-6):

    def error(theta):
        predicted_wrenches = measurement_matrix(theta, orientations)
        return matrix_error(wrenches, predicted_wrenches)

    return scipy.optimize.fmin(
        error, initial_theta, xtol=tolerance, ftol=tolerance**2, maxfun=10e3,
        maxiter=5e3)
