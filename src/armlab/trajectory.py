"""单段关节直线路径的五次时间缩放。"""

import numpy as np

from armlab._validation import positive_scalars


def quintic_trajectory(
    q_start, q_goal,
    velocity_limit=1.0,
    acceleration_limit=2.0,
    sample_count=101,
):
    """返回 times、positions、velocities、accelerations。

    输入姿态为长度为 2 的数组，单位 rad；两个正标量限制分别为
    rad/s、rad/s²，并共同用于两关节。输出形状为 (N,) 及三个 (N, 2)。
    按五次曲线的理论峰值选择时长，不检查路径碰撞或执行器力矩。
    起终点完全相同时返回 N 个零时间、重复位置及零速度、零加速度。
    """
    q_start = np.asarray(q_start, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)
    if q_start.shape != (2,) or q_goal.shape != (2,):
        raise ValueError("q_start and q_goal must have shape (2,)")
    if not np.all(np.isfinite(q_start)) or not np.all(np.isfinite(q_goal)):
        raise ValueError("joint angles must be finite")
    positive_scalars(
        velocity_limit=velocity_limit,
        acceleration_limit=acceleration_limit,
    )
    if (
        isinstance(sample_count, (bool, np.bool_))
        or not isinstance(sample_count, (int, np.integer))
        or sample_count < 2
    ):
        raise ValueError("sample_count must be an integer of at least 2")

    delta_q = q_goal - q_start
    if np.all(delta_q == 0.0):
        times = np.zeros(sample_count)
        positions = np.repeat(q_start[None, :], sample_count, axis=0)
        return times, positions, np.zeros_like(positions), np.zeros_like(positions)

    T_velocity = 1.875 * np.max(np.abs(delta_q)) / velocity_limit
    T_acceleration = np.sqrt(
        (10 / np.sqrt(3)) * np.max(np.abs(delta_q)) / acceleration_limit
    )
    T = max(T_velocity, T_acceleration)
    times = np.linspace(0.0, T, sample_count)
    positions = []
    velocities = []
    accelerations = []

    for t in times:
        u = t / T
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        ds_du = 30 * u**2 - 60 * u**3 + 30 * u**4
        d2s_du2 = 60 * u - 180 * u**2 + 120 * u**3
        q = q_start + s * delta_q
        q_dot = ds_du * delta_q / T
        q_ddot = d2s_du2 * delta_q / T**2
        positions.append(q.copy())
        velocities.append(q_dot.copy())
        accelerations.append(q_ddot.copy())

    return (
        times,
        np.array(positions),
        np.array(velocities),
        np.array(accelerations),
    )
