"""Compare J @ q_dot with the original forward time difference."""
import numpy as np
from armlab.forward_kinematics import forward_kinematics
from armlab.jacobian import jacobian


def main():
    q = np.deg2rad([30.0, 45.0])
    q_dot = np.deg2rad([10.0, -5.0])
    J = jacobian(*q, 0.30, 0.25)
    velocity = J @ q_dot
    dt = 1e-6
    before = forward_kinematics(*q, 0.30, 0.25)[:2, 3]
    after = forward_kinematics(*(q + q_dot * dt), 0.30, 0.25)[:2, 3]
    numerical = (after - before) / dt
    print("Joint 1 contribution (m/s):", J[:, 0] * q_dot[0])
    print("Joint 2 contribution (m/s):", J[:, 1] * q_dot[1])
    print("Tip velocity (m/s):", velocity)
    print("Finite difference (m/s):", numerical)
    print("Error (m/s):", np.linalg.norm(numerical - velocity))


if __name__ == "__main__":
    main()
