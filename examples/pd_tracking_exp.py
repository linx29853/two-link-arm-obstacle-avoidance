
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from armlab.pd_tracking import simulate_pd_tracking


def main():
    root = Path(__file__).resolve().parents[1]
    result = simulate_pd_tracking(
        root / "configs/two_link_arm.xml",
        np.deg2rad([0.0, 0.0]), np.deg2rad([-75.0, -150.0]),
    )
    moving = result["times"] <= result["trajectory_duration"]
    error = result["errors"][moving]
    print("轨迹时长（s）：", result["trajectory_duration"])
    print("运动阶段 RMSE（rad）：", np.sqrt(np.mean(error**2, axis=0)))
    print("运动阶段最大绝对误差（rad）：", np.max(np.abs(error), axis=0))
    print("稳定等待后的终点误差（rad）：", result["final_position_error"])
    print("实际指令峰值（N·m）：", np.max(np.abs(result["applied_torques"]), axis=0))
    print("限幅比例：", np.mean(np.abs(result["raw_torques"]) > 1.0, axis=0))
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    for j in range(2):
        axes[0].plot(result["times"], result["positions"][:, j], label=f"q{j+1} actual")
        axes[0].plot(result["times"], result["reference_positions"][:, j], "--", label=f"q{j+1} reference")
        axes[1].plot(result["times"], result["errors"][:, j], label=f"q{j+1}")
        axes[2].plot(result["times"], result["applied_torques"][:, j], label=f"joint {j+1}")
    for ax, label in zip(axes, ("Position (rad)", "Error (rad)", "Torque (N m)")):
        ax.set_ylabel(label)
        ax.axvline(result["trajectory_duration"], color="gray", linestyle=":")
        ax.grid(True)
        ax.legend()
    axes[-1].set_xlabel("Time (s)")
    fig.tight_layout()
    output = root / "assets/control/pd-tracking"
    output.mkdir(parents=True, exist_ok=True)
    fig.savefig(output / "pd-tracking.png", dpi=150)
    plt.close(fig)
    print("曲线保存至：", output / "pd-tracking.png")


if __name__ == "__main__":
    main()
