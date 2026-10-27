import numpy as np
from astar import (
    build_valid_grid,
    node_to_angles,
    astar_search,
)
from edge_check import is_edge_valid

def main():
    """设置实验参数、生成地图，再检查两个已知节点。"""
    angles_deg = np.array([-150, -75, 0, 75, 150])
    a1, a2 = 0.30, 0.25
    xc, yc = 0.15, 0.08
    r, rod_radius = 0.04, 0.01
    # 当前相邻节点差 75°，76 个点对应 1° 检查间隔。
    sample_count = 76

    valid_grid = build_valid_grid(angles_deg, a1, a2, xc, yc, r, rod_radius)

    start=(2,2)
    goal = (0, 1)
    path = astar_search(
        start, goal, valid_grid, angles_deg,
        a1, a2, xc, yc, r, rod_radius,
        sample_count=sample_count,
    )

    if path is None:
        print("没有返回路径")
    else:
        print("找到的路径：", path)
        for index in range(len(path) - 1):
            node_start = path[index]
            node_end = path[index + 1]

            q_from = node_to_angles(node_start, angles_deg)
            q_to = node_to_angles(node_end, angles_deg)

            edge_valid = is_edge_valid(
                q_from, q_to,
                sample_count=sample_count,
                a1=a1, a2=a2,
                xc=xc, yc=yc,
                r=r, rod_radius=rod_radius,
            )

            print(
                f"连接 {node_start} → {node_end}："
                f"复核结果={edge_valid}"
            )

if __name__ == "__main__":
    main()
