"""Small shared input checks for planar kinematics."""
import numpy as np


def finite_scalars(**values):
    for name, value in values.items():
        if np.ndim(value) != 0 or not np.isfinite(value):
            raise ValueError(f"{name} must be a finite scalar")


def positive_scalars(**values):
    finite_scalars(**values)
    for name, value in values.items():
        if value <= 0:
            raise ValueError(f"{name} must be positive")
