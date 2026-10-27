import numpy as np
from is_pose_valid import is_pose_valid

def is_edge_valid(q_start, q_goal, sample_count,
                    a1, a2, xc, yc, r, rod_radius):
    if sample_count < 2:
        raise ValueError("sample_count 至少为 2")
    for s in np.linspace(0.0,1.0,sample_count):
        q = (1 - s) * q_start + s * q_goal
        valid = is_pose_valid(
            q[0], q[1], a1, a2, xc, yc, r, rod_radius
        )
        
        if not valid:
           print(f"首次发现无效：s={s:.2f}, q={np.rad2deg(q)} deg")
           return False
    return True
