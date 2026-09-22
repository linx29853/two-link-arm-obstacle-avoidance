import numpy as np
from armlab.is_pose_valid import is_pose_valid
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

if __name__ == "__main__":
    a1 = 0.30          # 第一杆长度，m
    a2 = 0.25          # 第二杆长度，m
    xc, yc = 0.15, 0.08  # 障碍物圆心，m
    r = 0.04           # 障碍物半径，m
    rod_radius = 0.01  # 杆宽的一半，m
    angles_deg = np.arange(-150, 151, 5)
    n = len(angles_deg)
    valid_grid = np.zeros((n, n), dtype=bool)
    for i, q1_deg in enumerate(angles_deg):
        for j, q2_deg in enumerate(angles_deg):
            q1_rad = np.deg2rad(q1_deg)
            q2_rad = np.deg2rad(q2_deg)
            valid = is_pose_valid( q1_rad, q2_rad, a1, a2, xc, yc, r, rod_radius)
            valid_grid[j, i] = valid
    print("总姿态数：", valid_grid.size)
    print("有效姿态数：", np.count_nonzero(valid_grid))
    print("碰撞姿态数：", np.count_nonzero(~valid_grid))
    fig, ax = plt.subplots()
    # False=0 显示红色，True=1 显示浅蓝色
    cmap = ListedColormap(["tomato", "lightblue"])

    im = ax.imshow(
        valid_grid,
        origin="lower",
        cmap=cmap,
        vmin=0,
        vmax=1,
        interpolation="nearest",
    )

    # 格子的位置是下标，标签显示对应角度
    tick_indices = np.arange(0, n, 6)
    ax.set_xticks(tick_indices, labels=angles_deg[tick_indices])
    ax.set_yticks(tick_indices, labels=angles_deg[tick_indices])

    ax.set_xlabel("q1 (deg)")
    ax.set_ylabel("q2 (deg)")
    ax.set_title("Configuration space: sampled poses")

    colorbar = fig.colorbar(im, ax=ax, ticks=[0, 1])
    colorbar.ax.set_yticklabels(["Collision", "Valid"])

    plt.tight_layout()
    plt.show()
