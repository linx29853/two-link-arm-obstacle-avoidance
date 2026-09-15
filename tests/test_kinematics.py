"""Migrated checks plus AI-assisted regression checks; not learning sign-off."""
import contextlib
import importlib
import io

import numpy as np
import pytest

from armlab.forward_kinematics import forward_kinematics
from armlab.inverse_kinematics import inverse_kinematics
from armlab.jacobian import jacobian


@pytest.mark.parametrize("target", [
    [0.32451238, 0.39148146], [0.55, 0.0], [0.05, 0.0],
    [-0.2, 0.3], [-0.2, -0.3], [0.2, -0.3],
])
def test_analytic_branches_round_trip(target):
    solutions = inverse_kinematics(*target, 0.30, 0.25)
    assert len(solutions) == 2
    for q in solutions:
        actual = forward_kinematics(*q, 0.30, 0.25)[:2, 3]
        np.testing.assert_allclose(actual, target, atol=1e-9, rtol=0)


@pytest.mark.parametrize("target", [[0.60, 0.0], [0.0, 0.0]])
def test_unreachable_target_raises(target):
    with pytest.raises(ValueError, match="unreachable"):
        inverse_kinematics(*target, 0.30, 0.25)


@pytest.mark.parametrize("q", [[0.3, 0.7], [-1.2, 0.5], [0, 0], [0.2, np.pi]])
def test_jacobian_matches_columnwise_central_difference(q):
    q = np.asarray(q, dtype=float)
    h = 1e-6
    columns = []
    for axis in np.eye(2):
        plus = forward_kinematics(*(q + h * axis), 0.30, 0.25)[:2, 3]
        minus = forward_kinematics(*(q - h * axis), 0.30, 0.25)[:2, 3]
        columns.append((plus - minus) / (2 * h))
    np.testing.assert_allclose(jacobian(*q, 0.30, 0.25),
                               np.column_stack(columns), atol=1e-9, rtol=0)


def test_original_velocity_experiment():
    q = np.deg2rad([30.0, 45.0])
    q_dot = np.deg2rad([10.0, -5.0])
    dt = 1e-6
    before = forward_kinematics(*q, 0.30, 0.25)[:2, 3]
    after = forward_kinematics(*(q + q_dot * dt), 0.30, 0.25)[:2, 3]
    assert np.linalg.norm((after - before) / dt - jacobian(*q, 0.30, 0.25) @ q_dot) < 1e-6


@pytest.mark.parametrize("bad", [np.nan, np.inf])
def test_nonfinite_input_rejected(bad):
    with pytest.raises(ValueError):
        inverse_kinematics(bad, 0, 0.30, 0.25)
    with pytest.raises(ValueError):
        forward_kinematics(bad, 0, 0.30, 0.25)
    with pytest.raises(ValueError):
        jacobian(0, bad, 0.30, 0.25)


@pytest.mark.parametrize("length", [0, -0.3, np.nan])
def test_invalid_link_length_rejected(length):
    for function in (forward_kinematics, inverse_kinematics, jacobian):
        with pytest.raises(ValueError):
            function(0, 0, length, 0.25)


def test_imports_are_silent():
    output = io.StringIO()
    with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
        for name in ("forward_kinematics", "inverse_kinematics", "jacobian", "inverse_jacobian"):
            importlib.reload(importlib.import_module("armlab." + name))
    assert output.getvalue() == ""
