"""Display the existing forward-kinematics experiment."""
import numpy as np
from armlab.forward_kinematics import forward_kinematics


def main():
    T02 = forward_kinematics(*np.deg2rad([0, 90]), 0.30, 0.25)
    print("Tip transform:")
    print(T02)
    print("Tip XY (m):", T02[:2, 3])


if __name__ == "__main__":
    main()
