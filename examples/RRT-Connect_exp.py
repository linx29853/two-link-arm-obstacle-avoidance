"""调用 RRT-Connect 规划器，展示路径并逐段采样复核。"""
import numpy as np
from RRT_Connect import rrt_connect
from edge_check import is_edge_valid

def simplify_path(
    path, a1, a2, xc, yc, r, rod_radius,
    max_angle_step=np.deg2rad(1.0),
):
    

    if not path:
        return []

    simplified_path = [path[0].copy()]
    i = 0

    while i < len(path) - 1:
        next_index = i + 1  # 原路径已验证，相邻连接作为保底

        for j in range(len(path) - 1, i + 1, -1):
            q_from = path[i]
            q_to = path[j]

            direct_sample_count = max(2,int(np.ceil(np.max(np.abs(q_to - q_from)) / max_angle_step)) + 1,)
            direct_valid = is_edge_valid(
                q_from, q_to,
                sample_count=direct_sample_count,
                a1=a1, a2=a2,
                xc=xc, yc=yc,
                r=r, rod_radius=rod_radius,
            )

            if direct_valid:
                next_index = j
                break

        simplified_path.append(path[next_index].copy())
        i = next_index

    return simplified_path

def main():
    q_start = np.deg2rad([0.0, 0.0])
    q_goal = np.deg2rad([-75.0, -150.0])
    a1, a2 = 0.30, 0.25
    xc, yc = 0.15, 0.08
    r, rod_radius = 0.04, 0.01
    step_size = np.deg2rad(10.0)
    sample_count = 11
    max_iterations = 1000
    seed = 42

    path = rrt_connect(
        q_start, q_goal,
        a1=a1, a2=a2,
        xc=xc, yc=yc,
        r=r, rod_radius=rod_radius,
        step_size=step_size,
        sample_count=sample_count,
        max_iterations=max_iterations,
        seed=seed,
    )

    if path is None:
        print("本次未找到路径：起终点无效，或达到尝试上限")
        return

    print("找到路径，节点数：", len(path))
    print("路径角度（度）：", np.rad2deg(path))
    all_valid = True
    for i in range(len(path) - 1):
        edge_valid = is_edge_valid(
            path[i], path[i + 1],
            sample_count=sample_count,
            a1=a1, a2=a2,
            xc=xc, yc=yc,
            r=r, rod_radius=rod_radius,
        )
        if not edge_valid:
            all_valid = False
            print("无效连接：", i, "→", i + 1)

    print("整条路径采样复核：", all_valid)

    simplified_path = simplify_path(
        path, a1, a2, xc, yc, r, rod_radius
    )

    print("简化前节点数：", len(path))
    print("简化后节点数：", len(simplified_path))

if __name__ == "__main__":
    main()
