"""Numerical IK convergence, budget and invalid-input regression checks."""
import numpy as np
import pytest

from armlab.forward_kinematics import forward_kinematics
from armlab.inverse_jacobian import inverse_kinematics_numerical


def test_original_numerical_experiment_preserves_inputs(capsys):
    target = np.array([0.20, 0.40])
    initial = np.deg2rad([20.0, 60.0])
    saved_target, saved_initial = target.copy(), initial.copy()
    q, success, updates = inverse_kinematics_numerical(target, initial, 0.30, 0.25)
    assert success
    assert 0 < updates <= 200
    assert np.linalg.norm(forward_kinematics(*q, 0.30, 0.25)[:2, 3] - target) < 1e-6
    np.testing.assert_array_equal(initial, saved_initial)
    np.testing.assert_array_equal(target, saved_target)
    assert capsys.readouterr().out == ""


@pytest.mark.parametrize("at_target", [True, False])
def test_zero_budget_checks_initial_position(at_target):
    initial = np.array([0.3, 0.7])
    target = forward_kinematics(*initial, 0.30, 0.25)[:2, 3]
    if not at_target:
        target = target + [0.01, 0]
    q, success, updates = inverse_kinematics_numerical(
        target, initial, 0.30, 0.25, max_iterations=0)
    np.testing.assert_array_equal(q, initial)
    assert success == at_target
    assert updates == 0


def test_last_update_is_checked():
    initial = np.array([0.3, 0.7])
    target = forward_kinematics(*(initial + [0.001, -0.001]), 0.30, 0.25)[:2, 3]
    q, success, updates = inverse_kinematics_numerical(
        target, initial, 0.30, 0.25, learning_rate=1, max_iterations=1)
    assert success and updates == 1
    assert np.linalg.norm(forward_kinematics(*q, 0.30, 0.25)[:2, 3] - target) < 1e-6


def test_singular_start_reports_failure():
    q, success, updates = inverse_kinematics_numerical(
        [0.4, 0], [0, 0], 0.30, 0.25, max_iterations=5)
    assert not success
    assert updates == 5
    np.testing.assert_array_equal(q, [0, 0])


@pytest.mark.parametrize("override", [
    {"max_iterations": -1}, {"max_iterations": 1.5}, {"max_iterations": True},
    {"target": [0.2]}, {"target": [[0.2], [0.4]]}, {"target": [np.nan, 0.4]},
    {"q_init": [0]}, {"q_init": [0, np.inf]},
    {"learning_rate": 0}, {"tolerance": -1}, {"a1": 0},
])
def test_invalid_parameters(override):
    args = dict(target=[0.2, 0.4], q_init=[0.3, 0.7], a1=0.30, a2=0.25)
    args.update(override)
    with pytest.raises(ValueError):
        inverse_kinematics_numerical(**args)
