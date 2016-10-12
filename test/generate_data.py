import math
import random

from g_compensation.calibrate import measurement_matrix


def random_rpy():
    """ http://www.cognitive-antics.net/uniform-random-orientation/ """
    x, y, z = (random.random() for _ in range(3))
    roll = 2 * math.pi * x
    pitch = math.asin(y)
    yaw = math.pi * (2 * z - 1)
    return roll, pitch, yaw


def dataset(theta, samples, g=(0, 0, -9.81)):
    """
    Args:
        - theta: (see calibrate.py)
        - samples: number of generated measurements
    """
    os = [random_rpy() for _ in range(samples)]
    truth = measurement_matrix(theta, os)
    return os, truth
