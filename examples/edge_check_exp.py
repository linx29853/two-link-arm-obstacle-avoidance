import numpy as np
from armlab.is_pose_valid import is_pose_valid
from armlab.edge_check import is_edge_valid

if __name__ == "__main__":
    q_start = np.deg2rad([0.0, 0.0])
    q_goal = np.deg2rad([60.0, 0.0])

    a1, a2 = 0.30, 0.25
    xc, yc = 0.15, 0.08
    r = 0.04
    rod_radius = 0.01

    for sample_count in [2, 3, 11, 61]:
        valid = is_edge_valid(
            q_start, q_goal, sample_count,
            a1, a2, xc, yc, r, rod_radius,
        )
        print(f"采样点数={sample_count}，连接检查结果={valid}")