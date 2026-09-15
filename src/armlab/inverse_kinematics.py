"""Analytic geometric IK, without joint-limit filtering."""
import numpy as np

from ._validation import finite_scalars, positive_scalars


def solve_q2(x, y, a1, a2):
    """Return positive/negative elbow branches in radians."""
    finite_scalars(x=x, y=y)
    positive_scalars(a1=a1, a2=a2)
    c2 = (x**2 + y**2 - a1**2 - a2**2) / (2.0 * a1 * a2)
    tolerance = 1e-12
    if c2 < -1.0 - tolerance or c2 > 1.0 + tolerance:
        raise ValueError("Target is geometrically unreachable")
    c2 = np.clip(c2, -1.0, 1.0)
    s2 = np.sqrt(1.0 - c2**2)
    return np.arctan2(s2, c2), np.arctan2(-s2, c2)


def solve_q1(x, y, a1, a2, q2):
    """Return shoulder angle in radians for the selected elbow branch."""
    finite_scalars(x=x, y=y, q2=q2)
    positive_scalars(a1=a1, a2=a2)
    target_angle = np.arctan2(y, x)
    correction_angle = np.arctan2(a2 * np.sin(q2), a1 + a2 * np.cos(q2))
    return target_angle - correction_angle


def inverse_kinematics(x, y, a1, a2):
    """Return two (q1, q2) branches; raise ValueError if unreachable.

    Lengths are metres and angles radians. Branches may coincide at
    singularities. Equal links folded to the origin have infinitely many
    shoulder solutions; this function returns representative solutions.
    Angles are not wrapped and the project's +/-150 degree limits are
    not enforced here.
    """
    q2_positive, q2_negative = solve_q2(x, y, a1, a2)
    return [
        (solve_q1(x, y, a1, a2, q2_positive), q2_positive),
        (solve_q1(x, y, a1, a2, q2_negative), q2_negative),
    ]
