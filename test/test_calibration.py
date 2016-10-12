import random

from g_compensation.calibrate import estimate

from generate_data import dataset

estimate_tolerance = 1e-9
test_tolerance = 1e-6
default_initial_theta = (1, 0, 0, 0)
default_samples = 100


def random_theta():
    return (10e-3 + 5 * random.random(), -0.5 + random.random(),
            -0.5 + random.random(), -0.5 + random.random())


def run(true_theta, n_samples, initial_theta):
    o, w = dataset(true_theta, n_samples)
    result = estimate(o, w, initial_theta, tolerance=estimate_tolerance)
    assert all(abs(x - y) < test_tolerance for x, y in zip(
        true_theta, result)), 'Parameter estimation failed {} ({})'.format(
        result, true_theta)


def test_minimum_samples():
    for x in range(10):
        run(random_theta(),
            len(default_initial_theta),
            default_initial_theta)


def test_low_mass_high_offset():
    run((10e-3, 1, 1, 1),
        default_samples,
        default_initial_theta)


def test_high_mass_low_offset():
    run((100, 0.1e-3, 0.1e-3, 0.1e-3),
        default_samples,
        default_initial_theta)


def test_skewed_offset():
    run((2, 0.1e-3, 0.1e-3, 1),
        default_samples,
        default_initial_theta)
