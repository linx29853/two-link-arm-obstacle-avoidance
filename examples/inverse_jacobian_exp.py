"""Display the existing numerical IK experiment."""
import numpy as np
from armlab.forward_kinematics import forward_kinematics
from armlab.inverse_jacobian import inverse_kinematics_numerical


def main():
    target = np.array([0.20, 0.40])
    q, success, updates = inverse_kinematics_numerical(
        target, np.deg2rad([20.0, 60.0]), 0.30, 0.25,
    )
    actual = forward_kinematics(*q, 0.30, 0.25)[:2, 3]
    print("Converged:", success, "updates:", updates)
    print("q (deg):", np.rad2deg(q))
    print("Target XY (m):", target)
    print("Actual XY (m):", actual)
    print("Error (m):", np.linalg.norm(target - actual))


if __name__ == "__main__":
    main()
