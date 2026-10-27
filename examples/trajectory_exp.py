"""单段五次轨迹实验：自动选时长，展示状态并检查约束。"""

import numpy as np

from trajectory import quintic_trajectory


def main():
    q_start = np.deg2rad([0.0, 0.0])
    q_goal = np.deg2rad([-75.0, -150.0])
    velocity_limit = 1.0      # rad/s，教学设定
    acceleration_limit = 2.0  # rad/s²，教学设定

    times, positions, velocities, accelerations = quintic_trajectory(
        q_start, q_goal,
        velocity_limit=velocity_limit,
        acceleration_limit=acceleration_limit,
        sample_count=101,
    )

    # 当前奇数个采样点包含中间时刻，打印首、中、尾。
    for i in (0, len(times) // 2, len(times) - 1):
        print(
            f"t={times[i]:.2f} s: "
            f"q={np.rad2deg(positions[i])} deg, "
            f"q_dot={np.rad2deg(velocities[i])} deg/s, "
            f"q_ddot={np.rad2deg(accelerations[i])} deg/s²"
        )

    delta_q = q_goal - q_start
    T_velocity = 1.875 * np.max(np.abs(delta_q)) / velocity_limit
    T_acceleration = np.sqrt(
        (10 / np.sqrt(3)) * np.max(np.abs(delta_q)) / acceleration_limit
    )
    peak_speed = np.max(np.abs(velocities), axis=0)
    peak_acceleration = np.max(np.abs(accelerations), axis=0)
    tolerance = 1e-12  # 仅用于浮点比较，不是物理安全裕量

    print("速度要求时长：", T_velocity)
    print("加速度要求时长：", T_acceleration)
    print("选定总时长：", times[-1])
    print("采样速度检查：", np.all(peak_speed <= velocity_limit + tolerance))
    print("采样加速度检查：", np.all(peak_acceleration <= acceleration_limit + tolerance))
    print("各关节采样峰值速度（rad/s）：", peak_speed)
    print("各关节采样峰值加速度（rad/s²）：", peak_acceleration)
    print("保存数量：", len(positions), len(velocities), len(accelerations))


if __name__ == "__main__":
    main()
