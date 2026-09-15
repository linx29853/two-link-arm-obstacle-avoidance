"""Display analytic branches, boundary cases and unreachable targets."""
import numpy as np
from armlab.forward_kinematics import forward_kinematics
from armlab.inverse_kinematics import inverse_kinematics


def main():
    targets = [[0.32451238, 0.39148146], [0.55, 0], [0.05, 0], [0.60, 0], [0, 0]]
    print("Geometric IK only; joint limits are not enforced.")
    for target in targets:
        print("Target XY (m):", target)
        try:
            solutions = inverse_kinematics(*target, 0.30, 0.25)
        except ValueError as error:
            print(error)
            continue
        for index, q in enumerate(solutions, start=1):
            actual = forward_kinematics(*q, 0.30, 0.25)[:2, 3]
            print("Branch", index, "q (deg):", np.rad2deg(q))
            print("Actual XY (m):", actual, "error (m):", np.linalg.norm(actual - target))


if __name__ == "__main__":
    main()
